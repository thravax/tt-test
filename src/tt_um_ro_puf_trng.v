// RO-PUF + TRNG for TinyTapeout (IHP SG13G2)
//
// 8 ring oscillators (ROs), modelled as clock-divided flip-flops.
// Pairs are compared for PUF bits.
//
// Modes (uio_in[4:3]):
//   00 = RO-PUF : compare 4 pairs → 4 PUF bits in result[3:0]
//   10 = TRNG   : 8 measurement rounds, XOR counter LSBs → 1 byte
//   01, 11 = reserved (treated as RO-PUF)
//
// Serial interface (matches TinyTapeout convention):
//   Write (we=1, addr auto-increments):
//     addr 0 : window[7:0]  — measurement duration (clock cycles, default 64)
//   Read (re=1, addr auto-increments):
//     addr 0 : result       — 4 PUF bits (result[3:0]) or TRNG byte
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

    localparam NUM_ROS  = 8;
    localparam CNT_BITS = 8;

    // --- Control inputs ---
    wire       we       = uio_in[0];
    wire       re       = uio_in[1];
    wire       start    = uio_in[2];
    wire [1:0] mode     = uio_in[4:3];
    wire       addr_clr = uio_in[5];

    // --- Mode constants ---
    localparam MODE_RO_PUF = 2'b00;
    localparam MODE_TRNG   = 2'b10;

    // --- FSM ---
    localparam IDLE    = 2'd0;
    localparam MEASURE = 2'd1;
    localparam DONE    = 2'd2;
    reg [1:0] fsm;

    // --- Config register ---
    reg [7:0] window;     // measurement window (clock cycles)

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
    // Ring oscillators — behavioral clock-divided model.
    // RO[i] half-period = (i + 2) cycles.
    // Distinct, deterministic frequencies for simulation and synthesis.
    // On actual silicon the frequency ordering reflects process variation.
    // ---------------------------------------------------------------
    (* keep *) wire [NUM_ROS-1:0] ro_wire;

    generate
        genvar gi;
        for (gi = 0; gi < NUM_ROS; gi = gi + 1) begin : ro_gen
            reg [4:0] ro_div;
            reg       ro_sig;
            wire [4:0] half_p = gi[4:0] + 5'd2;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    ro_div <= 0;
                    ro_sig <= 0;
                end else if (ro_en[gi]) begin
                    if (ro_div >= half_p - 1) begin
                        ro_div <= 0;
                        ro_sig <= ~ro_sig;
                    end else begin
                        ro_div <= ro_div + 1;
                    end
                end else begin
                    ro_sig <= 0;
                    ro_div <= 0;
                end
            end
            assign ro_wire[gi] = ro_sig;
        end
    endgenerate

    // ---------------------------------------------------------------
    // Edge detection and counters (clk-domain sampling of ROs)
    // ---------------------------------------------------------------
    reg [CNT_BITS-1:0] cnt [0:NUM_ROS-1];
    reg [NUM_ROS-1:0]  ro_prev;

    // ---------------------------------------------------------------
    // PUF comparison: 4 pairs (RO0 vs RO1, RO2 vs RO3, RO4 vs RO5, RO6 vs RO7)
    // ---------------------------------------------------------------
    wire [3:0] puf_bits;
    generate
        genvar pi;
        for (pi = 0; pi < 4; pi = pi + 1) begin : puf_cmp
            assign puf_bits[pi] = (cnt[2*pi] > cnt[2*pi+1]);
        end
    endgenerate

    // ---------------------------------------------------------------
    // TRNG: XOR of all 8 counter LSBs
    // ---------------------------------------------------------------
    wire trng_new_bit;
    assign trng_new_bit = ^{cnt[0][0], cnt[1][0], cnt[2][0], cnt[3][0],
                            cnt[4][0], cnt[5][0], cnt[6][0], cnt[7][0]};

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
                            5'd0: window <= ui_in;
                            default: ;
                        endcase
                        addr <= addr + 1;
                    end else if (re) begin
                        case (addr)
                            5'd0:    uo_out <= result;
                            default: uo_out <= 8'hFF;
                        endcase
                        addr <= addr + 1;
                    end

                    if (start) begin
                        for (i = 0; i < NUM_ROS; i = i + 1)
                            cnt[i] <= 0;
                        ro_prev      <= ro_wire;
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
                        // Window expired — process result
                        if (mode_latch == MODE_TRNG) begin
                            trng_shift   <= {trng_shift[5:0], trng_new_bit};
                            trng_bit_cnt <= trng_bit_cnt + 1;
                            if (trng_bit_cnt == 3'd7) begin
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
                            // RO-PUF: 4 pairs → 4 bits in result[3:0]
                            result <= {4'b0, puf_bits};
                            fsm    <= DONE;
                        end
                    end
                end

                default: fsm <= IDLE;
            endcase
        end
    end

endmodule
