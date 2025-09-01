// Enhanced area-optimized 16-bit adex_neuron_system_tt_lut32.v
// Improved AdEx dynamics for multiple firing patterns
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

// Loader FSM
localparam L_IDLE=3'd0, L_SHIFT=3'd1, L_LATCH=3'd2, L_WAIT_FOOTER=3'd3, L_READY=3'd4;
reg [2:0]  lstate;
reg [7:0]  byte_acc;
reg        nibble_cnt;
reg [3:0]  param_idx;
reg [11:0] watchdog_cnt;
parameter WATCHDOG_MAX = 12'd4000;
parameter FOOTER_NIB = 4'b1111;

reg load_prev;
always @(posedge clk) load_prev <= reset ? 1'b0 : load_enable;

wire load_rising = load_enable && !load_prev;

// Parameter storage
reg [7:0] params [0:7];
reg r_ready;
reg [6:0] uo_out_reg;
assign uo_out = {1'b0, uo_out_reg};
wire [7:0] p_DeltaT = params[0], p_TauW = params[1], p_a = params[2], p_b = params[3];
wire [7:0] p_Vreset = params[4], p_VT = params[5], p_Ibias = params[6], p_C = params[7];
wire params_ready = r_ready;

// Loader FSM
always @(posedge clk) begin
    if (reset) begin
        lstate <= L_IDLE; byte_acc <= 8'd0; nibble_cnt <= 1'b0;
        param_idx <= 4'd0; watchdog_cnt <= 12'd0; r_ready <= 1'b0;
        // Default parameters that should produce spiking
        params[0] <= 8'd130; params[1] <= 8'd100; params[2] <= 8'd1; params[3] <= 8'd5;
        params[4] <= 8'd63;  params[5] <= 8'd78;  params[6] <= 8'd180; params[7] <= 8'd10;
    end else begin
        if (lstate != L_IDLE && watchdog_cnt < WATCHDOG_MAX) watchdog_cnt <= watchdog_cnt + 1'b1;
        else if (lstate != L_IDLE) begin lstate <= L_IDLE; nibble_cnt <= 1'b0; param_idx <= 4'd0; watchdog_cnt <= 12'd0; end

        case (lstate)
            L_IDLE: if (load_mode && load_rising) begin
                lstate <= L_SHIFT; nibble_cnt <= 1'b0; byte_acc <= 8'd0; param_idx <= 4'd0; watchdog_cnt <= 12'd0;
            end
            L_SHIFT: begin
                if (load_rising) begin
                    if (!nibble_cnt) begin byte_acc[7:4] <= nibble_in; nibble_cnt <= 1'b1; end
                    else begin byte_acc[3:0] <= nibble_in; nibble_cnt <= 1'b0; lstate <= L_LATCH; end
                    watchdog_cnt <= 12'd0;
                end
                if (!load_mode) begin lstate <= L_IDLE; nibble_cnt <= 1'b0; param_idx <= 4'd0; end
            end
            L_LATCH: begin
                params[param_idx] <= byte_acc;
                if (param_idx == 4'd7) lstate <= L_WAIT_FOOTER;
                else begin param_idx <= param_idx + 1'b1; lstate <= L_SHIFT; end
            end
            L_WAIT_FOOTER: if (load_rising) begin
                if (nibble_in == FOOTER_NIB) begin r_ready<=1'b1; lstate <= L_READY;
                end else lstate <= L_IDLE;
            end
            L_READY: if (!load_mode) begin r_ready <= 1'b0; lstate <= L_IDLE; end
            default: lstate <= L_IDLE;
        endcase
    end
end

// ---------------------- Enhanced 16-bit Core Neuron Logic ----------------------
reg signed [15:0] V, w, dV, dW, V_plus;
reg spike_reg;
reg [2:0] refrac_cnt;  // refractory period counter
reg signed [15:0] exp_val;         // Exponential value
reg signed [15:0] exp_current;     // Exponential current
reg signed [15:0] adaptation_term; // Adaptation term

localparam signed [15:0] gL_nS = 16'sd10 <<< 8;
localparam signed [15:0] EL_mV = -16'sd70 <<< 8;

reg signed [15:0] temp_calc, leak_term, exp_arg;
reg [7:0] vm8_reg, w8_reg;

// Enhanced exponential LUT with better resolution
function signed [15:0] exp_q_enhanced(input signed [15:0] arg_in);
    integer idx; 
    reg signed [15:0] val; 
    reg signed [15:0] RANGE_MIN, RANGE_MAX; 
    reg signed [31:0] tcalc, range_diff;
    
    RANGE_MIN = -16'sd6 <<< 8; 
    RANGE_MAX = 16'sd6 <<< 8;
    
    if(arg_in < RANGE_MIN) idx = 0;
    else if(arg_in > RANGE_MAX) idx = 31;
    else begin 
        tcalc = arg_in - RANGE_MIN; 
        range_diff = RANGE_MAX - RANGE_MIN; 
        idx = (tcalc * 32) / range_diff; 
    end
    
    case(idx)
        0: val=6;    1: val=9;    2: val=14;   3: val=21;
        4: val=31;   5: val=47;   6: val=71;   7: val=107;
        8: val=162;  9: val=245;  10: val=372; 11: val=564;
        12: val=855; 13: val=1296;14: val=1964;15: val=2978;
        16: val=4515;17: val=6844;18: val=10376;19: val=15728;
        20: val=23850;21: val=32767;22: val=32767;23: val=32767;
        24: val=32767;25: val=32767;26: val=32767;27: val=32767;
        28: val=32767;29: val=32767;30: val=32767;31: val=32767;
        default: val=32767;
    endcase
    exp_q_enhanced = val;
endfunction

// Enhanced multiply with better precision
function signed [15:0] qmul_enhanced(input signed [15:0] a, input signed [15:0] b);
    reg signed [31:0] temp_mult;
    temp_mult = a * b;
    qmul_enhanced = temp_mult >>> 8;
endfunction

// Enhanced division with saturation
function signed [15:0] qdiv_enhanced(input signed [15:0] a, input signed [15:0] b);
    reg signed [31:0] temp_div;
    if (b == 16'sd0) qdiv_enhanced = 16'sd0;
    else if (b[15] && (b > -16'sd4)) qdiv_enhanced = a[15] ? 16'sd32767 : -16'sd32768; // avoid div by tiny negative
    else if (!b[15] && (b < 16'sd4)) qdiv_enhanced = a[15] ? -16'sd32768 : 16'sd32767; // avoid div by tiny positive
    else begin
        temp_div = (a <<< 8) / b;
        if (temp_div > 32767) qdiv_enhanced = 16'sd32767;
        else if (temp_div < -32768) qdiv_enhanced = -16'sd32768;
        else qdiv_enhanced = temp_div[15:0];
    end
endfunction

// ---------------------- Helper Functions ----------------------
function signed [15:0] u8_to_signed_q_direct(input [7:0] x);
    u8_to_signed_q_direct = ($signed({8'b0, x}) - 16'sd128) <<< 8;
endfunction

function signed [15:0] u8_to_q_unsigned_direct(input [7:0] x);
    u8_to_q_unsigned_direct = $signed({8'b0, x}) <<< 8;
endfunction

function [7:0] sat_to_u8_fixed(input signed [15:0] x);
    reg signed [15:0] u; 
    u = (x >>> 8) + 16'sd128; 
    if(u < 0) u = 0; 
    if(u > 255) u = 255; 
    sat_to_u8_fixed = u[7:0];
endfunction

// ---------------------- Enhanced Compute FSM ----------------------
reg [2:0] compute_state;
localparam C_LEAK=0, C_ARG=1, C_EXP=2, C_DRIVE=3, C_DV=4, C_DW=5, C_UPDATE=6;

always @(posedge clk) begin
    if(reset) begin
        V <= -16'sd65 <<< 8; 
        w <= 16'sd0; 
        spike_reg <= 1'b0; 
        temp_calc <= 16'sd0;
        exp_val <= 16'sd0;
        exp_current <= 16'sd0;
        adaptation_term <= 16'sd0;
        leak_term <= 16'sd0;
        exp_arg <= 16'sd0;
        dV <= 16'sd0;
        dW <= 16'sd0;
        refrac_cnt <= 3'd0;
        compute_state <= C_LEAK; 
        vm8_reg <= 8'd63; 
        w8_reg <= 8'd128;
    end else begin
        if(enable_core && refrac_cnt == 3'd0) begin
            case(compute_state)
                C_LEAK: begin 
                    // Leak term: gL * (EL - V)
                    leak_term <= qmul_enhanced(gL_nS, EL_mV - V); 
                    compute_state <= C_ARG; 
                end
                
                C_ARG: begin 
                    // Exponential argument: (V - VT) / DeltaT
                    exp_arg <= qdiv_enhanced(V - u8_to_signed_q_direct(p_VT), u8_to_signed_q_direct(p_DeltaT));
                    compute_state <= C_EXP; 
                end
                
                C_EXP: begin 
                    // Calculate exponential value
                    exp_val <= exp_q_enhanced(exp_arg);
                    compute_state <= C_DRIVE; 
                end
                
                C_DRIVE: begin 
                    // Exponential current: gL * DeltaT * exp_val
                    exp_current <= qmul_enhanced(qmul_enhanced(gL_nS, u8_to_signed_q_direct(p_DeltaT)), exp_val);
                    compute_state <= C_DV; 
                end
                
                C_DV: begin 
                    // Total current and dV calculation
                    temp_calc <= leak_term + exp_current - w + u8_to_q_unsigned_direct(p_Ibias);
                    compute_state <= C_DW; 
                end
                
                C_DW: begin 
                    // Calculate dV and adaptation term
                    dV <= qdiv_enhanced(temp_calc, u8_to_q_unsigned_direct(p_C));
                    adaptation_term <= qmul_enhanced(u8_to_q_unsigned_direct(p_a), V - EL_mV);
                    compute_state <= C_UPDATE; 
                end
                
                C_UPDATE: begin
                    // Calculate dW
                    dW <= qdiv_enhanced(adaptation_term - w, u8_to_q_unsigned_direct(p_TauW));
                    
                    // Check for spike and update
                    if((V + dV) > u8_to_signed_q_direct(p_VT)) begin
                        spike_reg <= 1'b1; 
                        V <= u8_to_signed_q_direct(p_Vreset); 
                        w <= w + dW + u8_to_q_unsigned_direct(p_b);
                        refrac_cnt <= 3'd1;
                    end else begin
                        spike_reg <= 1'b0;
                        V <= V + dV; 
                        w <= w + dW;
                    end
                    
                    // Update output registers
                    vm8_reg <= sat_to_u8_fixed(V);
                    w8_reg  <= sat_to_u8_fixed(w);
                    compute_state <= C_LEAK;
                end
                default: compute_state <= C_LEAK;
            endcase
        end else begin
            if(refrac_cnt > 3'd0) begin
                refrac_cnt <= refrac_cnt - 1'b1;
                spike_reg <= 1'b0;
            end
            if(!enable_core) compute_state <= C_LEAK;
        end
    end
end

// Output assignment
always @(*) begin
    uo_out_reg[0] = spike_reg;
    if(!debug_mode) uo_out_reg[6:1] = vm8_reg[7:2];
    else uo_out_reg[6:1] = w8_reg[7:2];
end

endmodule