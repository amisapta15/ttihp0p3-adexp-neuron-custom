// Area-optimized 16-bit adex_neuron_system_tt_lut32.v - Fixed lint and timing
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

// Parameter loader - Fixed width issues
localparam L_IDLE=3'd0, L_SHIFT=3'd1, L_LATCH=3'd2, L_WAIT_FOOTER=3'd3, L_READY=3'd4;
reg [2:0]  lstate;
reg [7:0]  byte_acc;
reg        nibble_cnt;
reg [2:0]  param_idx;  // Fixed: reduced from 4 bits to 3 bits (0-7 only needs 3 bits)
reg [11:0] watchdog_cnt;
parameter WATCHDOG_MAX = 12'd4000;
parameter FOOTER_NIB = 4'b1111;
reg        load_prev;

// Parameter register bank
reg [7:0] params [0:7];
reg       r_ready;
reg  [6:0] uo_out_reg;
assign uo_out = {1'b0, uo_out_reg};

initial begin
    uo_out_reg = 7'b0;
end

// Removed unused parameter alias wires to save area
// Access params array directly instead of through unused wires

always @(posedge clk) begin
    if (reset) begin
        lstate <= L_IDLE; byte_acc <= 8'd0; nibble_cnt <= 1'b0;
        param_idx <= 3'd0; watchdog_cnt <= 12'd0; r_ready <= 1'b0; load_prev <= 1'b0;
        // Initialize parameter array with defaults
        params[0] <= 8'd130;   // DeltaT = 2mV
        params[1] <= 8'd228;   // TauW = 100ms  
        params[2] <= 8'd130;   // a = 2nS
        params[3] <= 8'd168;   // b = 40pA
        params[4] <= 8'd63;    // Vreset = -65mV
        params[5] <= 8'd78;    // VT = -50mV
        params[6] <= 8'd143;   // Ibias = 15pA
        params[7] <= 8'd200;   // C = 200pF
    end else begin
        load_prev <= load_enable;
        if (lstate != L_IDLE) begin
            if (watchdog_cnt < WATCHDOG_MAX) watchdog_cnt <= watchdog_cnt + 1'b1;
            else begin lstate <= L_IDLE; nibble_cnt <= 1'b0; param_idx <= 3'd0; watchdog_cnt <= 12'd0; end
        end
        case (lstate)
            L_IDLE: if (load_mode && load_enable && !load_prev) begin 
                lstate <= L_SHIFT; nibble_cnt <= 1'b0; byte_acc <= 8'd0; param_idx <= 3'd0; watchdog_cnt <= 12'd0; 
            end
            L_SHIFT: begin
                if (load_enable && !load_prev) begin
                    if (nibble_cnt == 1'b0) begin byte_acc[7:4] <= nibble_in; nibble_cnt <= 1'b1; end
                    else begin byte_acc[3:0] <= nibble_in; nibble_cnt <= 1'b0; lstate <= L_LATCH; end
                    watchdog_cnt <= 12'd0;
                end
                if (!load_mode) begin lstate <= L_IDLE; nibble_cnt <= 1'b0; param_idx <= 3'd0; end
            end
            L_LATCH: begin
                params[param_idx] <= byte_acc;
                if (param_idx == 3'd7) lstate <= L_WAIT_FOOTER;
                else begin param_idx <= param_idx + 1'b1; lstate <= L_SHIFT; end
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

// ---------------------- 16-bit Core Neuron Logic ----------------------
reg signed [15:0] V, w;
reg spike_reg;

// Fixed constants in Q4.8 format
localparam signed [15:0] gL_nS = (16'sd10)  <<< 8;      // 10 * 2^8
localparam signed [15:0] EL_mV = (-16'sd70) <<< 8;      // -70 * 2^8

// Single shared intermediate register - Fixed width declarations
reg signed [15:0] temp_result;  // Renamed to avoid conflict
reg [7:0] vm8_reg, w8_reg;

// 16-bit Q arithmetic helpers - Fixed width issues
function signed [15:0] qmul;
    input signed [15:0] a, b;
    reg signed [31:0] temp_mul;  // Renamed to avoid conflict
    begin
        temp_mul = $signed(a) * $signed(b);
        qmul = temp_mul[23:8];  // Fixed: proper bit extraction instead of shift
    end
endfunction

function signed [15:0] qdiv;
    input signed [15:0] a, b;
    reg signed [31:0] temp_div;  // Renamed to avoid conflict
    begin
        if (b == 16'sd0) qdiv = 16'sd0;
        else begin
            temp_div = ($signed(a) <<< 8);
            qdiv = temp_div / $signed(b);
        end
    end
endfunction

// Optimized exponential function - Fixed variable naming
function signed [15:0] exp_q;
    input signed [15:0] arg_in;
    integer idx; 
    reg signed [15:0] val;
    reg signed [15:0] RANGE_MIN, RANGE_MAX;
    reg signed [31:0] temp_exp, range_diff;  // Renamed to avoid conflict
    begin
        RANGE_MIN = (-16'sd4)<<<8; 
        RANGE_MAX = (16'sd4)<<<8;
        if(arg_in < RANGE_MIN) idx = 0;
        else if(arg_in > RANGE_MAX) idx = 31;
        else begin
            temp_exp = $signed(arg_in) - $signed(RANGE_MIN);
            range_diff = $signed(RANGE_MAX) - $signed(RANGE_MIN);
            idx = (temp_exp * 32) / range_diff;
            if (idx > 31) idx = 31;
            if (idx < 0) idx = 0;
        end
        case(idx)
            0: val=18;   1: val=25;   2: val=33;   3: val=45;
            4: val=61;   5: val=82;   6: val=111;  7: val=150;
            8: val=203;  9: val=275;  10: val=372; 11: val=503;
           12: val=681; 13: val=921; 14: val=1245;15: val=1684;
           16: val=2279;17: val=3084;18: val=4171;19: val=5644;
           20: val=7634;21: val=10332;22: val=13975;23: val=18906;
           24: val=25575;25: val=32767;26: val=32767;27: val=32767;
           28: val=32767;29: val=32767;30: val=32767;31: val=32767;
           default: val = 32767;
        endcase
        exp_q = val;
    end
endfunction

// Inline parameter conversion functions
function signed [15:0] u8_to_signed_q_direct;
    input [7:0] x;
    begin
        u8_to_signed_q_direct = ($signed({8'b0, x}) - 16'sd128) <<< 8;
    end
endfunction

function signed [15:0] u8_to_q_unsigned_direct;
    input [7:0] x;
    begin
        u8_to_q_unsigned_direct = $signed({8'b0, x}) <<< 8;
    end
endfunction

function [7:0] sat_to_u8_fixed;
    input signed [15:0] x;
    reg signed [15:0] u;
    begin
        u = (x >>> 8) + 16'sd128;
        if (u < 16'sd0) u = 16'sd0;
        if (u > 16'sd255) u = 16'sd255;
        sat_to_u8_fixed = u[7:0];
    end
endfunction

// TIMING FIX: Add pipeline registers to break critical paths
reg [2:0] compute_state;
reg signed [15:0] leak_current, exp_term, drive_current, dV, dw;
reg signed [15:0] V_threshold, V_reset, tau_w, delta_t, ibias, capacitance, param_a, param_b;

localparam C_LEAK=0, C_ARG=1, C_EXP=2, C_DRIVE=3, C_DV=4, C_DW=5, C_UPDATE=6;

// Pre-convert parameters to reduce timing pressure
always @(posedge clk) begin
    if (reset) begin
        V_threshold <= (-16'sd50) <<< 8;
        V_reset <= (-16'sd65) <<< 8;
        tau_w <= (16'sd100) <<< 8;
        delta_t <= (16'sd2) <<< 8;
        ibias <= (16'sd15) <<< 8;
        capacitance <= (16'sd200) <<< 8;
        param_a <= (16'sd2) <<< 8;
        param_b <= (16'sd40) <<< 8;
    end else if (r_ready) begin
        // Update parameter cache when new params loaded
        V_threshold <= u8_to_signed_q_direct(params[5]);
        V_reset <= u8_to_signed_q_direct(params[4]);
        tau_w <= u8_to_q_unsigned_direct(params[1]);
        delta_t <= u8_to_signed_q_direct(params[0]);
        ibias <= u8_to_signed_q_direct(params[6]);
        capacitance <= u8_to_q_unsigned_direct(params[7]);
        param_a <= u8_to_q_unsigned_direct(params[2]);
        param_b <= u8_to_q_unsigned_direct(params[3]);
    end
end

always @(posedge clk) begin
    if (reset) begin
        V <= (-16'sd65) <<< 8;
        w <= 16'sd0; 
        spike_reg <= 1'b0;
        temp_result <= 16'sd0;
        compute_state <= C_LEAK;
        vm8_reg <= 8'd63;
        w8_reg <= 8'd128;
        leak_current <= 16'sd0;
        exp_term <= 16'sd0;
        drive_current <= 16'sd0;
        dV <= 16'sd0;
        dw <= 16'sd0;
    end else begin
        if (enable_core) begin
            case (compute_state)
                C_LEAK: begin
                    leak_current <= qmul(gL_nS, (EL_mV - V));
                    compute_state <= C_ARG;
                end
                C_ARG: begin
                    if (delta_t == 16'sd0) begin
                        exp_term <= 16'sd0;
                        compute_state <= C_DRIVE;  // Skip exponential
                    end else begin
                        temp_result <= qdiv((V - V_threshold), delta_t);
                        compute_state <= C_EXP;
                    end
                end
                C_EXP: begin
                    exp_term <= qmul(gL_nS, qmul(delta_t, exp_q(temp_result)));
                    compute_state <= C_DRIVE;
                end
                C_DRIVE: begin
                    drive_current <= leak_current + exp_term - w + ibias;
                    compute_state <= C_DV;
                end
                C_DV: begin
                    if (capacitance < (16'sd10 <<< 8)) begin
                        dV <= qdiv(drive_current, (16'sd10 <<< 8));
                    end else begin
                        dV <= qdiv(drive_current, capacitance);
                    end
                    compute_state <= C_DW;
                end
                C_DW: begin
                    if (tau_w < (16'sd1 <<< 8)) begin
                        dw <= 16'sd0;  // Skip dw calculation
                    end else begin
                        dw <= qdiv((qmul(param_a, (V - EL_mV)) - w), tau_w);
                    end
                    compute_state <= C_UPDATE;
                end
                C_UPDATE: begin
                    // Apply voltage update
                    V <= V + dV;
                    
                    // Spike detection and reset
                    if ((V + dV) > V_threshold) begin
                        spike_reg <= 1'b1;
                        V <= V_reset;
                        w <= w + dw + param_b;
                    end else begin
                        spike_reg <= 1'b0;
                        w <= w + dw;
                    end

                    // Simplified saturation - reduce timing pressure
                    if ((V + dV) < (-16'sd150 <<< 8)) V <= (-16'sd150 <<< 8);
                    else if ((V + dV) > (16'sd100 <<< 8)) V <= (16'sd100 <<< 8);
                    
                    if ((w + dw) < (-16'sd100 <<< 8)) w <= (-16'sd100 <<< 8);
                    else if ((w + dw) > (16'sd127 <<< 8)) w <= (16'sd127 <<< 8);
                    
                    // Update output registers
                    vm8_reg <= sat_to_u8_fixed(V + dV);
                    w8_reg  <= sat_to_u8_fixed(w + dw);
                    
                    compute_state <= C_LEAK;  // Loop back
                end
                default: compute_state <= C_LEAK;
            endcase
        end else begin
            compute_state <= C_LEAK;
            // Remove X-state checks that cause lint warnings
            vm8_reg <= 8'd63;
            w8_reg <= 8'd128;
        end
    end
end

// Simplified output logic
always @(*) begin
    uo_out_reg[0] = spike_reg;
    if (!debug_mode) uo_out_reg[6:1] = vm8_reg[7:2];
    else uo_out_reg[6:1] = w8_reg[7:2];
end

endmodule