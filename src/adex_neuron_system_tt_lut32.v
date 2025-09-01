/*
  File: adex_neuron_system_tt_lut32.v
  Compact Verilog-2001 implementation with 32-entry LUT for TinyTapeout (IHP 25b).
  - Optimized for smaller area (1x2 tile ≈ 202 µm × 313 µm)
  - Interface adapted for standard TinyTapeout wrapper.
  - Core: AdEx-like adaptive exponential IF, Q4.12 fixed point (signed 16-bit).
  - Outputs: top-6 bits of Vm (or w if debug_mode=1) on uo_out[6:1], spike on uo_out[0].
  - Written to be synthesizable with OpenLane (Verilog-2001, no SystemVerilog).
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
// Port unpacking
// -----------------------------------------------------------------------------
wire reset       = ~rst_n;      // reset is active-high internally
wire load_mode   = ui_in[4];
wire load_enable = ui_in[3];
wire enable_core = ui_in[2];
wire debug_mode  = ui_in[1];

// nibble inputs from uio[3:0] (external master must drive them during load_mode)
wire [3:0] nibble_in;
assign nibble_in = uio_in[3:0];

// This module only reads from uio, so set it to input-only.
assign uio_out = 8'b0;
assign uio_oe  = 8'b0; // 0=input, 1=output

// -----------------------------------------------------------------------------
// Loader: nibble-based FSM (IDLE, SHIFT, LATCH, WAIT_FOOTER, READY)
// -----------------------------------------------------------------------------
localparam L_IDLE        = 3'd0;
localparam L_SHIFT       = 3'd1;
localparam L_LATCH       = 3'd2;
localparam L_WAIT_FOOTER = 3'd3;
localparam L_READY       = 3'd4;

reg [2:0]  lstate;
reg [3:0]  nibble_buf;
reg [7:0]  byte_acc;
reg        nibble_cnt;        // 0 or 1
reg [2:0]  param_idx;        // 0..6
reg [15:0] watchdog_cnt;

parameter WATCHDOG_MAX = 16'd50000;
parameter FOOTER_NIB = 4'b1111;

reg        load_prev;

// staging (uncommitted) params
reg [7:0] s_DeltaT, s_TauW, s_a, s_b, s_Vreset, s_VT, s_Ibias;
// committed params (outputs from loader)
reg [7:0] r_DeltaT, r_TauW, r_a, r_b, r_Vreset, r_VT, r_Ibias;
reg       r_ready;

// outputs assignments
reg  [6:0] uo_out_reg;
assign uo_out = {1'b0, uo_out_reg}; // Pad 7-bit output to match 8-bit wrapper port

// map loader outputs to wires for core
wire [7:0] p_DeltaT = r_DeltaT;
wire [7:0] p_TauW   = r_TauW;
wire [7:0] p_a      = r_a;
wire [7:0] p_b      = r_b;
wire [7:0] p_Vreset = r_Vreset;
wire [7:0] p_VT     = r_VT;
wire [7:0] p_Ibias  = r_Ibias;
wire       params_ready = r_ready;

// -----------------------------------------------------------------------------
// Loader sequential logic
// -----------------------------------------------------------------------------
always @(posedge clk) begin
    if (reset) begin
        lstate <= L_IDLE;
        nibble_buf <= 4'd0;
        byte_acc <= 8'd0;
        nibble_cnt <= 1'b0;
        param_idx <= 3'd0;
        watchdog_cnt <= 16'd0;
        s_DeltaT <= 8'd0; s_TauW <= 8'd0; s_a <= 8'd0; s_b <= 8'd0;
        s_Vreset <= 8'd0; s_VT <= 8'd0; s_Ibias <= 8'd0;
        r_DeltaT <= 8'd0; r_TauW <= 8'd0; r_a <= 8'd0; r_b <= 8'd0;
        r_Vreset <= 8'd0; r_VT <= 8'd0; r_Ibias <= 8'd0;
        r_ready <= 1'b0;
        load_prev <= 1'b0;
    end else begin
        load_prev <= load_enable;

        // watchdog
        if (lstate != L_IDLE) begin
            if (watchdog_cnt < WATCHDOG_MAX) begin
                watchdog_cnt <= watchdog_cnt + 1'b1;
            end else begin
                // abort
                lstate <= L_IDLE;
                nibble_cnt <= 1'b0;
                param_idx <= 3'd0;
                watchdog_cnt <= 16'd0;
            end
        end

        case (lstate)
            L_IDLE: begin
                r_ready <= 1'b0;
                if (load_mode) begin
                    if (load_enable && !load_prev) begin
                        // start capture
                        lstate <= L_SHIFT;
                        nibble_cnt <= 1'b0;
                        nibble_buf <= 4'd0;
                        byte_acc <= 8'd0;
                        param_idx <= 3'd0;
                        watchdog_cnt <= 16'd0;
                    end
                end
            end

            L_SHIFT: begin
                // capture nibble on rising edge of load_enable
                if (load_enable && !load_prev) begin
                    nibble_buf <= nibble_in;
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
                // abort if load_mode deasserted
                if (!load_mode) begin
                    lstate <= L_IDLE;
                    nibble_cnt <= 1'b0;
                    param_idx <= 3'd0;
                end
            end

            L_LATCH: begin
                case (param_idx)
                    3'd0: s_DeltaT <= byte_acc;
                    3'd1: s_TauW   <= byte_acc;
                    3'd2: s_a      <= byte_acc;
                    3'd3: s_b      <= byte_acc;
                    3'd4: s_Vreset <= byte_acc;
                    3'd5: s_VT     <= byte_acc;
                    3'd6: s_Ibias  <= byte_acc;
                    default: ;
                endcase

                if (param_idx == 3'd6) begin
                    lstate <= L_WAIT_FOOTER;
                end else begin
                    param_idx <= param_idx + 1'b1;
                    lstate <= L_SHIFT;
                end
            end

            L_WAIT_FOOTER: begin
                // wait for nibble (rising load_enable) equal to footer
                if (load_enable && !load_prev) begin
                    if (nibble_in == FOOTER_NIB) begin
                        // commit staged params
                        r_DeltaT <= s_DeltaT;
                        r_TauW   <= s_TauW;
                        r_a      <= s_a;
                        r_b      <= s_b;
                        r_Vreset <= s_Vreset;
                        r_VT     <= s_VT;
                        r_Ibias  <= s_Ibias;
                        r_ready  <= 1'b1;
                        lstate <= L_READY;
                    end else begin
                        // invalid footer: abort
                        lstate <= L_IDLE;
                        nibble_cnt <= 1'b0;
                        param_idx <= 3'd0;
                    end
                end
            end

            L_READY: begin
                if (!load_mode) begin
                    r_ready <= 1'b0;
                    lstate <= L_IDLE;
                    param_idx <= 3'd0;
                    nibble_cnt <= 1'b0;
                end
            end

            default: lstate <= L_IDLE;
        endcase
    end
end

// -----------------------------------------------------------------------------
// Core variables and intermediate calculations
// -----------------------------------------------------------------------------
reg signed [15:0] V;    // Q4.12
reg signed [15:0] w;    // Q4.12
reg spike_reg;

// parameter Q4.12 representations
reg signed [15:0] DeltaT_q, TauW_q, a_q, b_q, Vreset_q, VT_q, Ibias_q;

// constants
localparam signed [15:0] C_pF  = (16'sd200) <<< 12; // 200 * 2^12
localparam signed [15:0] gL_nS = (16'sd10)  <<< 12; // 10 * 2^12
localparam signed [15:0] EL_mV = ( -16'sd70 ) <<< 12;    // -70 * 2^12

// intermediate calculation registers
reg signed [15:0] leak;
reg signed [15:0] arg;
reg signed [15:0] expterm;
reg signed [15:0] drive;
reg signed [15:0] dV;
reg signed [15:0] dw;

// output registers
reg [7:0] vm8_reg, w8_reg;

// helper arithmetic functions
function signed [15:0] qmul;
    input signed [15:0] a;
    input signed [15:0] b;
    reg signed [31:0] mul_result;
    begin
        mul_result = a * b; // 32-bit
        qmul = mul_result[27:12]; // Extract bits [27:12] for proper Q4.12 result
    end
endfunction

function signed [15:0] qdiv;
    input signed [15:0] a;
    input signed [15:0] b;
    reg signed [31:0] num;
    reg signed [31:0] res;
    reg signed [31:0] b_extended;
    begin
        if (b == 16'sd0) begin
            qdiv = 16'sd0;
        end else begin
            // Fix: Cast both 'a' and 'b' to 32 bits
            num = {{16{a[15]}}, a} <<< 12; // Sign-extend 'a' to 32 bits, then shift
            b_extended = {{16{b[15]}}, b}; // Sign-extend 'b' to 32 bits
            res = num / b_extended;
            qdiv = res[15:0]; // Extract lower 16 bits
        end
    end
endfunction

// enhanced exp() LUT: 32-entry for finer resolution
function signed [15:0] exp_q;
    input signed [15:0] arg_in; // Renamed to fix VARHIDDEN warning
    integer idx;
    reg signed [15:0] val;
    integer span;
    reg signed [15:0] RANGE_MIN;
    reg signed [15:0] RANGE_MAX;
    reg signed [31:0] temp_calc;
    reg signed [31:0] range_diff;
    begin
        RANGE_MIN = (-16'sd4) <<< 12;
        RANGE_MAX = ( 16'sd8) <<< 12;
        span = 32;
        if (arg_in < RANGE_MIN) begin
            idx = 0;
        end else if (arg_in > RANGE_MAX) begin
            idx = span - 1;
        end else begin
            // Fix bit-width issues in index calculation
            temp_calc = {{16{arg_in[15]}}, arg_in} - {{16{RANGE_MIN[15]}}, RANGE_MIN};
            range_diff = {{16{RANGE_MAX[15]}}, RANGE_MAX} - {{16{RANGE_MIN[15]}}, RANGE_MIN} + 32'sd1;
            idx = (temp_calc * span) / range_diff;
        end

        case (idx)
            0:  val = (16'sd0)    <<< 12;
            1:  val = (16'sd0)    <<< 12;
            2:  val = (16'sd1)    <<< 12;
            3:  val = (16'sd1)    <<< 12;
            4:  val = (16'sd1)    <<< 12;
            5:  val = (16'sd2)    <<< 12;
            6:  val = (16'sd2)    <<< 12;
            7:  val = (16'sd3)    <<< 12;
            8:  val = (16'sd4)    <<< 12;
            9:  val = (16'sd5)    <<< 12;
            10: val = (16'sd6)    <<< 12;
            11: val = (16'sd8)    <<< 12;
            12: val = (16'sd10)   <<< 12;
            13: val = (16'sd12)   <<< 12;
            14: val = (16'sd15)   <<< 12;
            15: val = (16'sd19)   <<< 12;
            16: val = (16'sd23)   <<< 12;
            17: val = (16'sd28)   <<< 12;
            18: val = (16'sd35)   <<< 12;
            19: val = (16'sd42)   <<< 12;
            20: val = (16'sd52)   <<< 12;
            21: val = (16'sd63)   <<< 12;
            22: val = (16'sd77)   <<< 12;
            23: val = (16'sd94)   <<< 12;
            24: val = (16'sd114)  <<< 12;
            25: val = (16'sd139)  <<< 12;
            26: val = (16'sd169)  <<< 12;
            27: val = (16'sd206)  <<< 12;
            28: val = (16'sd251)  <<< 12;
            29: val = (16'sd305)  <<< 12;
            30: val = (16'sd371)  <<< 12;
            31: val = (16'sd451)  <<< 12;
            default: val = (16'sd8700) <<< 12;
        endcase
        exp_q = val;
    end
endfunction

// convert 8-bit unsigned to signed Q4.12 with mid=128 where desired
function signed [15:0] u8_to_signed_q_mid;
    input [7:0] x;
    reg signed [15:0] tmp;
    begin
        tmp = $signed({8'b0, x}) - 16'sd128; // Convert to 16-bit signed, then subtract 128
        u8_to_signed_q_mid = {tmp[3:0], 12'b0}; // Shift left 12 bits safely
    end
endfunction

function signed [15:0] u8_to_q_unsigned;
    input [7:0] x;
    reg signed [15:0] tmp;
    begin
        tmp = $signed({8'b0, x}); // Convert to 16-bit signed
        u8_to_q_unsigned = {tmp[3:0], 12'b0}; // Shift left 12 bits safely
    end
endfunction

// saturation to 8-bit unsigned (returns [7:0])
function [7:0] sat_to_u8;
    input signed [15:0] x;
    reg signed [15:0] mv;
    reg signed [15:0] u;
    begin
        mv = x >>> 12; // back to integer mV (arithmetic shift for sign extension)
        u = mv + 16'sd128;
        if (u < 16'sd0) u = 16'sd0;
        if (u > 16'sd255) u = 16'sd255;
        sat_to_u8 = u[7:0];
    end
endfunction

// core sequential
always @(posedge clk) begin
    if (reset) begin
        DeltaT_q <= u8_to_signed_q_mid(8'd2);
        TauW_q   <= u8_to_q_unsigned(8'd100);
        a_q      <= u8_to_q_unsigned(8'd2);
        b_q      <= u8_to_q_unsigned(8'd40);
        Vreset_q <= u8_to_signed_q_mid(8'd191); // -65 encoded as 191 if using mid=128
        VT_q     <= u8_to_signed_q_mid(8'd206); // -50 -> 206
        Ibias_q  <= 16'sd0;
        V <= u8_to_signed_q_mid(8'd191);
        w <= 16'sd0;
        spike_reg <= 1'b0;
        leak <= 16'sd0;
        arg <= 16'sd0;
        expterm <= 16'sd0;
        drive <= 16'sd0;
        dV <= 16'sd0;
        dw <= 16'sd0;
        vm8_reg <= 8'd0;
        w8_reg <= 8'd0;
    end else begin
        // update params when loader committed
        if (params_ready) begin
            DeltaT_q <= u8_to_signed_q_mid(p_DeltaT);
            TauW_q   <= u8_to_q_unsigned(p_TauW);
            a_q      <= u8_to_q_unsigned(p_a);
            b_q      <= u8_to_q_unsigned(p_b);
            Vreset_q <= u8_to_signed_q_mid(p_Vreset);
            VT_q     <= u8_to_signed_q_mid(p_VT);
            Ibias_q  <= u8_to_signed_q_mid(p_Ibias);
        end

        if (enable_core) begin
            // AdEx neuron dynamics calculations
            leak <= qmul(gL_nS, (EL_mV - V));
            arg  <= qdiv((V - VT_q), DeltaT_q);
            expterm <= qmul(gL_nS, qmul(DeltaT_q, exp_q(arg)));
            drive <= leak + expterm - w + Ibias_q;
            dV <= qmul(qdiv(drive, C_pF), (16'sd1 <<< 12)); // dt=1ms scaled
            dw <= qmul(qdiv((qmul(a_q, (V - EL_mV)) - w), TauW_q), (16'sd1 <<< 12));

            V <= V + dV;
            w <= w + dw;

            // spike detection and reset
            if (V > (VT_q + (16'sd18 <<< 12))) begin
                spike_reg <= 1'b1;
                V <= Vreset_q;
                w <= w + b_q;
            end else begin
                spike_reg <= 1'b0;
            end

            // clamping to prevent overflow
            if (V > (16'sd100 <<< 12)) V <= (16'sd100 <<< 12);
            if (V < (-16'sd150 <<< 12)) V <= (-16'sd150 <<< 12);
            if (w > (16'sd500 <<< 12)) w <= (16'sd500 <<< 12);
            if (w < (-16'sd500 <<< 12)) w <= (-16'sd500 <<< 12);
        end
        
        // update output registers
        vm8_reg <= sat_to_u8(V);
        w8_reg  <= sat_to_u8(w);
    end
end

// -----------------------------------------------------------------------------
// Output muxing (6 MSBs of vm or w + spike bit)
// -----------------------------------------------------------------------------
wire [5:0] vm_top6 = vm8_reg[7:2];
wire [5:0] w_top6  = w8_reg[7:2];

always @(*) begin
    uo_out_reg[0] = spike_reg;
    if (!debug_mode) begin
        uo_out_reg[6:1] = vm_top6;
    end else begin
        uo_out_reg[6:1] = w_top6;
    end
end

endmodule