// RO-PUF + CRO-PUF + TRNG for TinyTapeout (IHP SG13G2)
//
// 16 configurable ring oscillators (CROs).
// Each RO has a 1-bit challenge input that bypasses 2 inverter stages,
// changing its frequency.  Pairs are compared for PUF bits.
//
// Modes (uio_in[4:3]):
//   00 = RO-PUF  : challenge=0, compare 8 pairs → 8 PUF bits
//   01 = CRO-PUF : challenge selects per-RO bypass → 8 PUF bits
//   10 = TRNG    : 8 measurement rounds, XOR counter LSBs → 1 byte
//   11 = reserved
//
// Serial interface (matches TinyTapeout convention):
//   Write (we=1, addr auto-increments):
//     addr 0 : window[7:0]      — measurement duration (clock cycles)
//     addr 1 : challenge[7:0]   — CRO config for ROs  0–7
//     addr 2 : challenge[15:8]  — CRO config for ROs 8–15
//   Read (re=1, addr auto-increments):
//     addr  0     : result      — 8 PUF bits or TRNG byte
//     addr  1–16  : cnt[0..15]  — raw 8-bit counter values
//
// Pin map:
//   ui_in[7:0]  = data_in
//   uo_out[7:0] = data_out
//   uio_in[0]   = we
//   uio_in[1]   = re
//   uio_in[2]   = start
//   uio_in[4:3] = mode
//   uio_in[5]   = addr_clr
//   uio_out[7]  = done
//   uio_out[6]  = busy
//   uio_oe      = 0xC0

`default_nettype none

module tt_um_ro_puf_trng (
    input  wire [7:0] ui_in,
    output reg  [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    localparam NUM_ROS  = 16;
    localparam CNT_BITS = 8;

    // --- Control inputs ---
    wire       we       = uio_in[0];
    wire       re       = uio_in[1];
    wire       start    = uio_in[2];
    wire [1:0] mode     = uio_in[4:3];
    wire       addr_clr = uio_in[5];

    // --- Mode constants ---
    localparam MODE_RO_PUF  = 2'b00;
    localparam MODE_CRO_PUF = 2'b01;
    localparam MODE_TRNG    = 2'b10;

    // --- FSM ---
    localparam IDLE    = 2'd0;
    localparam MEASURE = 2'd1;
    localparam DONE    = 2'd2;
    reg [1:0] fsm;

    // --- Config registers ---
    reg [7:0]           window;       // measurement window (clock cycles)
    reg [NUM_ROS-1:0]   challenge;    // per-RO bypass bit

    // --- Address counter ---
    reg [4:0] addr;

    // --- Measurement window counter ---
    reg [7:0] win_cnt;

    // --- Mode latched at start ---
    reg [1:0] mode_latch;

    // --- TRNG accumulation ---
    // trng_shift[6:0] holds bits from rounds 0..6 (oldest at [6]).
    // On round 7 result = {trng_shift[6:0], trng_new_bit}.
    reg [6:0] trng_shift;
    reg [2:0] trng_bit_cnt;

    // --- PUF / TRNG result ---
    reg [7:0] result;

    // --- RO enable: high only during MEASURE ---
    wire [NUM_ROS-1:0] ro_en = {NUM_ROS{(fsm == MEASURE)}};

    // ---------------------------------------------------------------
    // Ring oscillators (16 CROs)
    // ---------------------------------------------------------------
    (* keep, dont_touch *) wire [NUM_ROS-1:0] ro_wire;

`ifndef SYNTHESIS
    // Simulation model: clock-divided behavioral ROs.
    // RO[i] half-period = (i + 2) cycles, +2 extra if challenge[i]=1.
    // This gives deterministic, distinct frequencies for cocotb testing.
    generate
        genvar gi;
        for (gi = 0; gi < NUM_ROS; gi = gi + 1) begin : ro_sim_gen
            reg [4:0] ro_div;
            reg       ro_sim;
            wire [4:0] half_p = (gi[4:0] + 5'd2) + (challenge[gi] ? 5'd2 : 5'd0);
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    ro_div <= 0;
                    ro_sim <= 0;
                end else if (ro_en[gi]) begin
                    if (ro_div >= half_p - 1) begin
                        ro_div <= 0;
                        ro_sim <= ~ro_sim;
                    end else begin
                        ro_div <= ro_div + 1;
                    end
                end else begin
                    ro_sim <= 0;
                    ro_div <= 0;
                end
            end
            assign ro_wire[gi] = ro_sim;
        end
    endgenerate
`else
    // Synthesis: real ring oscillators with optional 2-stage extension.
    // challenge[gi]=0 → 5 inversions (NAND + 4 INV)
    // challenge[gi]=1 → 7 inversions (NAND + 4 INV + 2 INV via MUX)
    generate
        genvar gi;
        for (gi = 0; gi < NUM_ROS; gi = gi + 1) begin : ro_synth_gen
            (* keep *) wire n0, n1, n2, n3, n4, ext0, ext1, ro_fb;
            assign n0   = ~(ro_fb & ro_en[gi]);   // NAND: enable gate
            assign n1   = ~n0;
            assign n2   = ~n1;
            assign n3   = ~n2;
            assign n4   = ~n3;
            assign ext0 = ~n4;                     // optional 2 extra
            assign ext1 = ~ext0;
            assign ro_fb = challenge[gi] ? ext1 : n4;
            assign ro_wire[gi] = ro_fb;
        end
    endgenerate
`endif

    // ---------------------------------------------------------------
    // Edge detection and counters (clk-domain sampling of async ROs)
    // ---------------------------------------------------------------
    reg [CNT_BITS-1:0] cnt [0:NUM_ROS-1];
    reg [NUM_ROS-1:0]  ro_prev;

    // ---------------------------------------------------------------
    // PUF comparison: 8 pairs (RO0 vs RO1, RO2 vs RO3, ..., RO14 vs RO15)
    // ---------------------------------------------------------------
    wire [7:0] puf_bits;
    generate
        genvar pi;
        for (pi = 0; pi < 8; pi = pi + 1) begin : puf_cmp
            assign puf_bits[pi] = (cnt[2*pi] > cnt[2*pi+1]);
        end
    endgenerate

    // ---------------------------------------------------------------
    // TRNG: XOR of all 16 counter LSBs
    // ---------------------------------------------------------------
    wire trng_new_bit;
    assign trng_new_bit = ^{cnt[0][0],  cnt[1][0],  cnt[2][0],  cnt[3][0],
                            cnt[4][0],  cnt[5][0],  cnt[6][0],  cnt[7][0],
                            cnt[8][0],  cnt[9][0],  cnt[10][0], cnt[11][0],
                            cnt[12][0], cnt[13][0], cnt[14][0], cnt[15][0]};

    // ---------------------------------------------------------------
    // Outputs
    // ---------------------------------------------------------------
    assign uio_oe  = 8'hC0;
    assign uio_out = {(fsm == DONE), (fsm == MEASURE), 6'b0};

    // ---------------------------------------------------------------
    // Sequential logic — single always block to avoid multi-driver
    // ---------------------------------------------------------------
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm          <= IDLE;
            addr         <= 0;
            window       <= 8'd64;
            challenge    <= 0;
            win_cnt      <= 0;
            mode_latch   <= 0;
            trng_shift   <= 0;
            trng_bit_cnt <= 0;
            result       <= 0;
            uo_out       <= 0;
            ro_prev      <= 0;
            for (i = 0; i < NUM_ROS; i = i + 1)
                cnt[i] <= 0;
        end else begin

            // Track RO edges every cycle (used in MEASURE)
            ro_prev <= ro_wire;

            case (fsm)

                // ---------------------------------------------------------
                IDLE, DONE: begin
                    if (addr_clr) begin
                        addr <= 0;
                    end else if (we) begin
                        case (addr)
                            5'd0: window          <= ui_in;
                            5'd1: challenge[7:0]  <= ui_in;
                            5'd2: challenge[15:8] <= ui_in;
                            default: ;
                        endcase
                        addr <= addr + 1;
                    end else if (re) begin
                        case (addr)
                            5'd0:    uo_out <= result;
                            5'd1:    uo_out <= cnt[0];
                            5'd2:    uo_out <= cnt[1];
                            5'd3:    uo_out <= cnt[2];
                            5'd4:    uo_out <= cnt[3];
                            5'd5:    uo_out <= cnt[4];
                            5'd6:    uo_out <= cnt[5];
                            5'd7:    uo_out <= cnt[6];
                            5'd8:    uo_out <= cnt[7];
                            5'd9:    uo_out <= cnt[8];
                            5'd10:   uo_out <= cnt[9];
                            5'd11:   uo_out <= cnt[10];
                            5'd12:   uo_out <= cnt[11];
                            5'd13:   uo_out <= cnt[12];
                            5'd14:   uo_out <= cnt[13];
                            5'd15:   uo_out <= cnt[14];
                            5'd16:   uo_out <= cnt[15];
                            default: uo_out <= 8'hFF;
                        endcase
                        addr <= addr + 1;
                    end

                    if (start) begin
                        for (i = 0; i < NUM_ROS; i = i + 1)
                            cnt[i] <= 0;
                        ro_prev      <= ro_wire;   // capture current state
                        win_cnt      <= window - 1;
                        mode_latch   <= mode;
                        addr         <= 0;
                        trng_bit_cnt <= 0;
                        trng_shift   <= 0;
                        fsm          <= MEASURE;
                    end
                end

                // ---------------------------------------------------------
                MEASURE: begin
                    if (win_cnt > 0) begin
                        // Count rising edges while the window is open
                        for (i = 0; i < NUM_ROS; i = i + 1) begin
                            if (ro_wire[i] & ~ro_prev[i])
                                cnt[i] <= cnt[i] + 1;
                        end
                        win_cnt <= win_cnt - 1;
                    end else begin
                        // Window expired — process result.
                        // Note: cnt updates above via NB assignment are not
                        // yet visible to puf_bits/trng_new_bit (correct).
                        if (mode_latch == MODE_TRNG) begin
                            trng_shift   <= {trng_shift[5:0], trng_new_bit};
                            trng_bit_cnt <= trng_bit_cnt + 1;
                            if (trng_bit_cnt == 3'd7) begin
                                // trng_shift[6:0] = bits from rounds 0-6 (oldest at [6])
                                // trng_new_bit     = bit from round 7
                                result <= {trng_shift[6:0], trng_new_bit};
                                fsm    <= DONE;
                            end else begin
                                // Reset for next TRNG round
                                for (i = 0; i < NUM_ROS; i = i + 1)
                                    cnt[i] <= 0;
                                ro_prev <= ro_wire;
                                win_cnt <= window - 1;
                            end
                        end else begin
                            // RO-PUF or CRO-PUF
                            result <= puf_bits;
                            fsm    <= DONE;
                        end
                    end
                end

                default: fsm <= IDLE;
            endcase
        end
    end

endmodule
