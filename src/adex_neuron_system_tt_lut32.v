/*
  File: adex_neuron_system_tt_lut32.v
  Final golden Verilog-2001 implementation for Tiny Tapeout (IHP 25b).
  - This version incorporates a configurable membrane capacitance (C) as an 8th parameter.
*/
 // ============================================================================

module adex_neuron_system_tt_lut32 (
    input             clk,
    input             rst_n,
    input       [7:0] ui_in,
    output      [7:0] uo_out,
    input       [7:0] uio_in,
    output      [7:0] uio_out,
    output      [7:0] uio_oe
);

// -----------------------------------------------------------------------------
// Port unpacking & I/O Assignments
// -----------------------------------------------------------------------------
wire reset       = ~rst_n;      // Internal reset is active-high
wire load_mode   = ui_in[4];
wire load_enable = ui_in[3];
wire enable_core = ui_in[2];
wire debug_mode  = ui_in[1];

wire [3:0] nibble_in;
assign nibble_in = uio_in[3:0];

assign uio_out = 8'b0;
assign uio_oe  = 8'b0;

// -----------------------------------------------------------------------------
// Loader: nibble-based FSM
// -----------------------------------------------------------------------------
localparam L_IDLE        = 3'd0;
localparam L_SHIFT       = 3'd1;
localparam L_LATCH       = 3'd2;
localparam L_WAIT_FOOTER = 3'd3;
localparam L_READY       = 3'd4;

reg [2:0]  lstate;
reg [7:0]  byte_acc;
reg        nibble_cnt;
reg [3:0]  param_idx; // Widened to [3:0] for 8 parameters (0-7)
reg [15:0] watchdog_cnt;

parameter WATCHDOG_MAX = 16'd50000;
parameter FOOTER_NIB = 4'b1111;

reg        load_prev;

// Added s_C and r_C for the new capacitance parameter
reg [7:0] s_DeltaT, s_TauW, s_a, s_b, s_Vreset, s_VT, s_Ibias, s_C;
reg [7:0] r_DeltaT, r_TauW, r_a, r_b, r_Vreset, r_VT, r_Ibias, r_C;
reg       r_ready;

reg  [6:0] uo_out_reg;
assign uo_out = {1'b0, uo_out_reg};

// Added p_C wire for the new parameter
wire [7:0] p_DeltaT = r_DeltaT;
wire [7:0] p_TauW   = r_TauW;
wire [7:0] p_a      = r_a;
wire [7:0] p_b      = r_b;
wire [7:0] p_Vreset = r_Vreset;
wire [7:0] p_VT     = r_VT;
wire [7:0] p_Ibias  = r_Ibias;
wire [7:0] p_C      = r_C;
wire       params_ready = r_ready;

// -----------------------------------------------------------------------------
// Loader sequential logic
// -----------------------------------------------------------------------------
always @(posedge clk) begin
    if (reset) begin
        lstate <= L_IDLE;
        byte_acc <= 8'd0;
        nibble_cnt <= 1'b0;
        param_idx <= 4'd0;
        watchdog_cnt <= 16'd0;
        s_DeltaT <= 8'd0; s_TauW <= 8'd0; s_a <= 8'd0; s_b <= 8'd0;
        s_Vreset <= 8'd0; s_VT <= 8'd0; s_Ibias <= 8'd0; s_C <= 8'd0; // Reset new param
        r_DeltaT <= 8'd0; r_TauW <= 8'd0; r_a <= 8'd0; r_b <= 8'd0;
        r_Vreset <= 8'd0; r_VT <= 8'd0; r_Ibias <= 8'd0; r_C <= 8'd0; // Reset new param
        r_ready <= 1'b0;
        load_prev <= 1'b0;
    end else begin
        load_prev <= load_enable;

        if (lstate != L_IDLE) begin
            if (watchdog_cnt < WATCHDOG_MAX) begin
                watchdog_cnt <= watchdog_cnt + 1'b1;
            end else begin
                lstate <= L_IDLE;
                nibble_cnt <= 1'b0;
                param_idx <= 4'd0;
                watchdog_cnt <= 16'd0;
            end
        end

        case (lstate)
            L_IDLE: begin
                r_ready <= 1'b0;
                if (load_mode && load_enable && !load_prev) begin
                    lstate <= L_SHIFT;
                    nibble_cnt <= 1'b0;
                    byte_acc <= 8'd0;
                    param_idx <= 4'd0;
                    watchdog_cnt <= 16'd0;
                end
            end
            L_SHIFT: begin
                if (load_enable && !load_prev) begin
                    if (nibble_cnt == 1'b0) begin
                        byte_acc[7:4] <= nibble_in;
                        nibble_cnt <= 1'b1;
                    end else begin
                        byte_acc[3:0] <= nibble_in;
                        nibble_cnt <= 1'b0;
                        lstate <= L_LATCH;
                    end
                    watchdog_cnt <= 16'd0;
                end
                if (!load_mode) begin
                    lstate <= L_IDLE;
                    nibble_cnt <= 1'b0;
                    param_idx <= 4'd0;
                end
            end
            L_LATCH: begin
                case (param_idx)
                    4'd0: s_DeltaT <= byte_acc; 4'd1: s_TauW <= byte_acc;
                    4'd2: s_a      <= byte_acc; 4'd3: s_b    <= byte_acc;
                    4'd4: s_Vreset <= byte_acc; 4'd5: s_VT   <= byte_acc;
                    4'd6: s_Ibias  <= byte_acc; 4'd7: s_C    <= byte_acc; // Latch new param
                    default: ;
                endcase
                if (param_idx == 4'd7) lstate <= L_WAIT_FOOTER; // Condition updated to 7
                else begin
                    param_idx <= param_idx + 1'b1;
                    lstate <= L_SHIFT;
                end
            end
            L_WAIT_FOOTER: begin
                if (load_enable && !load_prev) begin
                    if (nibble_in == FOOTER_NIB) begin
                        r_DeltaT <= s_DeltaT; r_TauW <= s_TauW;
                        r_a <= s_a;           r_b <= s_b;
                        r_Vreset <= s_Vreset; r_VT <= s_VT;
                        r_Ibias <= s_Ibias;   r_C <= s_C; // Commit new param
                        r_ready <= 1'b1;
                        lstate <= L_READY;
                    end else lstate <= L_IDLE;
                end
            end
            L_READY: if (!load_mode) lstate <= L_IDLE;
            default: lstate <= L_IDLE;
        endcase
    end
end

// -----------------------------------------------------------------------------
// Core Neuron Logic
// -----------------------------------------------------------------------------
reg signed [15:0] V, w;
reg spike_reg;
reg signed [15:0] DeltaT_q, TauW_q, a_q, b_q, Vreset_q, VT_q, Ibias_q, C_q; // Added C_q
// localparam signed [15:0] C_pF  = (16'sd200) <<< 12; // REMOVED
localparam signed [15:0] gL_nS = (16'sd10)  <<< 12;
localparam signed [15:0] EL_mV = (-16'sd70) <<< 12;
reg signed [15:0] leak, arg, expterm, drive, dV, dw;
reg [7:0] vm8_reg, w8_reg;

function signed [15:0] qmul;
    input signed [15:0] a, b;
    begin qmul = (a * b) >>> 12; end
endfunction

function signed [15:0] qdiv;
    input signed [15:0] a, b;
    begin
        if (b == 0) qdiv = 0;
        else qdiv = (a <<< 12) / b;
    end
endfunction

function signed [15:0] exp_q;
    input signed [15:0] arg_in;
    integer idx;
    reg signed [15:0] val;
    reg signed [15:0] RANGE_MIN, RANGE_MAX;
    reg signed [31:0] temp_calc, range_diff;
    begin
        RANGE_MIN = (-16'sd4) <<< 12; RANGE_MAX = (16'sd8) <<< 12;
        if (arg_in < RANGE_MIN) idx = 0;
        else if (arg_in > RANGE_MAX) idx = 31;
        else begin
            temp_calc = {{16{arg_in[15]}}, arg_in} - {{16{RANGE_MIN[15]}}, RANGE_MIN};
            range_diff = {{16{RANGE_MAX[15]}}, RANGE_MAX} - {{16{RANGE_MIN[15]}}, RANGE_MIN} + 1;
            idx = (temp_calc * 32) / range_diff;
        end
        case (idx)
            0: val=(16'sd0); 1: val=(16'sd0); 2: val=(16'sd1); 3: val=(16'sd1);
            4: val=(16'sd1); 5: val=(16'sd2); 6: val=(16'sd2); 7: val=(16'sd3);
            8: val=(16'sd4); 9: val=(16'sd5); 10: val=(16'sd6); 11: val=(16'sd8);
            12: val=(16'sd10); 13: val=(16'sd12); 14: val=(16'sd15); 15: val=(16'sd19);
            16: val=(16'sd23); 17: val=(16'sd28); 18: val=(16'sd35); 19: val=(16'sd42);
            20: val=(16'sd52); 21: val=(16'sd63); 22: val=(16'sd77); 23: val=(16'sd94);
            24: val=(16'sd114); 25: val=(16'sd139); 26: val=(16'sd169); 27: val=(16'sd206);
            28: val=(16'sd251); 29: val=(16'sd305); 30: val=(16'sd371); 31: val=(16'sd451);
            default: val = (16'sd451);
        endcase
        exp_q = val <<< 12;
    end
endfunction

function signed [15:0] u8_to_signed_q_mid;
    input [7:0] x;
    reg signed [15:0] tmp;
    begin
        tmp = $signed({8'b0, x}) - 16'sd128;
        u8_to_signed_q_mid = tmp <<< 12;
    end
endfunction

function signed [15:0] u8_to_q_unsigned;
    input [7:0] x;
    begin
        u8_to_q_unsigned = $signed({8'b0, x}) <<< 12;
    end
endfunction

function [7:0] sat_to_u8;
    input signed [15:0] x;
    reg signed [15:0] u;
    begin
        u = (x >>> 12) + 16'sd128;
        if (u < 0) u = 0;
        if (u > 255) u = 255;
        sat_to_u8 = u[7:0];
    end
endfunction

// -----------------------------------------------------------------------------
// Core sequential logic
// -----------------------------------------------------------------------------
always @(posedge clk) begin
    if (reset) begin
        V <= u8_to_signed_q_mid(8'd63);
        w <= 16'sd0;
        spike_reg <= 1'b0;
        DeltaT_q <= u8_to_signed_q_mid(8'd130);
        TauW_q   <= u8_to_q_unsigned(8'd100);
        a_q      <= u8_to_q_unsigned(8'd2);
        b_q      <= u8_to_q_unsigned(8'd40);
        Vreset_q <= u8_to_signed_q_mid(8'd63);
        VT_q     <= u8_to_signed_q_mid(8'd78);
        Ibias_q  <= 16'sd0;
        C_q      <= u8_to_q_unsigned(8'd200); // Initialize C_q on reset
    end else begin
        if (params_ready) begin
            DeltaT_q <= u8_to_signed_q_mid(p_DeltaT); TauW_q   <= u8_to_q_unsigned(p_TauW);
            a_q      <= u8_to_q_unsigned(p_a);      b_q      <= u8_to_q_unsigned(p_b);
            Vreset_q <= u8_to_signed_q_mid(p_Vreset); VT_q     <= u8_to_signed_q_mid(p_VT);
            Ibias_q  <= u8_to_signed_q_mid(p_Ibias);  C_q      <= u8_to_q_unsigned(p_C); // Update C_q
        end

        if (enable_core) begin
            leak <= qmul(gL_nS, (EL_mV - V));
            arg  <= qdiv((V - VT_q), DeltaT_q);
            expterm <= qmul(gL_nS, qmul(DeltaT_q, exp_q(arg)));
            drive <= leak + expterm - w + Ibias_q;
            dV <= qdiv(drive, C_q); // Use configurable C_q
            dw <= qdiv((qmul(a_q, (V - EL_mV)) - w), TauW_q);

            V <= V + dV;
            w <= w + dw;

            if (V > VT_q) begin
                spike_reg <= 1'b1;
                V <= Vreset_q;
                w <= w + b_q;
            end else spike_reg <= 1'b0;

            if (V > ( 16'sd100 <<< 12)) V <= ( 16'sd100 <<< 12);
            if (V < (-16'sd150 <<< 12)) V <= (-16'sd150 <<< 12);
            if (w > ( 16'sd500 <<< 12)) w <= ( 16'sd500 <<< 12);
            if (w < (-16'sd500 <<< 12)) w <= (-16'sd500 <<< 12);
        end
        
        vm8_reg <= sat_to_u8(V);
        w8_reg  <= sat_to_u8(w);
    end
end

// -----------------------------------------------------------------------------
// Output Mux
// -----------------------------------------------------------------------------
always @(*) begin
    uo_out_reg[0] = spike_reg;
    if (!debug_mode) uo_out_reg[6:1] = vm8_reg[7:2];
    else uo_out_reg[6:1] = w8_reg[7:2];
end

endmodule