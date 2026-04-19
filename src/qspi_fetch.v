// qspi_fetch.v — TinyCPU-8 instruction-fetch unit driving an external W25Q128-
//                class QSPI flash in Fast Read Quad I/O mode (SPEC v1.1 §4).
//
// 7-state FSM: IDLE, CMD, ADDR, MODE, DUMMY, READ_H, READ_L.
// (SPEC §4.3's DONE is implicit — READ_L collapses directly back to READ_H
//  for continuous streaming. `fetch_valid` is the 1-cycle published-word pulse.)
//
// Timing (SPI mode 0, CPU clock / 2):
//   * `sck_r` is a REGISTERED output that toggles every clk while the FSM is
//     active (any state != IDLE). It IS `qspi_sck`.
//   * Using a register avoids the combinational-gating glitch that would
//     occur if we AND'd `half` with a state-based gate.
//   * Rising-edge boundary = the posedge where pre-edge `sck_r==0` (i.e.,
//     the one where sck_r is flipping to 1). State, counter, and input-sample
//     updates happen here.
//   * Output `qspi_d_out` is a COMBINATIONAL mux over (state, ctr) indexing
//     into a constant-byte/word register (cmd_byte, addr_word, mode_byte).
//     No shift register ⇒ no cross-edge race.
//
// Per-phase SPI-cycle counts:
//     CMD   : 8 cycles (x1 on D0, MSB first)
//     ADDR  : 6 cycles (x4, MSB nibble of 24-bit byte address first)
//     MODE  : 2 cycles (x4, 0x00 -- no continuous-read; CMD re-sent every branch)
//     DUMMY : 4 cycles (master tristated; slave turns bus around on last cycle)
//     READ_H: 2 cycles (sample 2 nibbles → high byte)
//     READ_L: 2 cycles (sample 2 nibbles → low byte; publishes IR)
//
// Byte-address (SPEC §4.4): byte_addr = {11'b0, pc_addr, 1'b0}.
// Redirect (SPEC §4.5): aborts in-flight fetch; IDLE → new CMD with fresh pc_addr.
// Halted (SPEC §3.8): aborts in-flight fetch immediately; IDLE kickoff gated off.

`default_nettype none

module qspi_fetch (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [11:0] pc_addr,
    input  wire        redirect,
    input  wire        halted,

    output reg  [15:0] ir_out,
    output reg         fetch_valid,

    output wire        qspi_cs_n,
    output wire        qspi_sck,
    output wire [3:0]  qspi_d_out,
    output wire [3:0]  qspi_d_oe,
    input  wire [3:0]  qspi_d_in,

    output reg         heartbeat
);

    localparam [2:0]
        S_IDLE   = 3'd0,
        S_CMD    = 3'd1,
        S_ADDR   = 3'd2,
        S_MODE   = 3'd3,
        S_DUMMY  = 3'd4,
        S_READ_H = 3'd5,
        S_READ_L = 3'd6;

    localparam [7:0] CMD_BYTE  = 8'hEB;
    localparam [7:0] MODE_BYTE = 8'h00;   // non-continuous-read

    reg [2:0]  state;
    reg        sck_r;
    reg [3:0]  ctr;
    reg [23:0] addr_word;        // captured byte address for this fetch
    reg [15:0] rx_word;

    // ── Wire-up of QSPI outputs ─────────────────────────────────────────
    assign qspi_cs_n = (state == S_IDLE);
    assign qspi_sck  = sck_r;

    reg [3:0] d_oe_c, d_out_c;
    always @(*) begin
        d_oe_c  = 4'b0000;
        d_out_c = 4'b0000;
        case (state)
            S_CMD: begin
                d_oe_c  = 4'b0001;
                d_out_c = {3'b0, (CMD_BYTE >> (4'd7 - ctr)) & 8'h01};
            end
            S_ADDR: begin
                d_oe_c  = 4'b1111;
                d_out_c = (addr_word >> (5'd20 - {1'b0, ctr, 2'b0})) & 24'hF;
            end
            S_MODE: begin
                d_oe_c  = 4'b1111;
                d_out_c = (MODE_BYTE >> (4'd4 - {ctr, 2'b0})) & 8'hF;
            end
            default: ;
        endcase
    end
    assign qspi_d_out = d_out_c;
    assign qspi_d_oe  = d_oe_c;

    // ── FSM ──────────────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            sck_r       <= 1'b0;
            ctr         <= 4'd0;
            addr_word   <= 24'h0;
            rx_word     <= 16'h0;
            ir_out      <= 16'hFFFF;
            fetch_valid <= 1'b0;
            heartbeat   <= 1'b0;
        end else begin
            fetch_valid <= 1'b0;

            if (redirect || halted) begin
                state <= S_IDLE;
                sck_r <= 1'b0;
                ctr   <= 4'd0;
            end
            else if (state == S_IDLE) begin
                sck_r <= 1'b0;
                if (!halted) begin
                    state     <= S_CMD;
                    addr_word <= {11'b0, pc_addr, 1'b0};
                    ctr       <= 4'd0;
                end
            end
            else begin
                // Active states: toggle SCK.
                sck_r <= ~sck_r;

                if (sck_r == 1'b0) begin
                    // Rising-edge boundary.
                    case (state)
                        S_CMD: begin
                            if (ctr == 4'd7) begin state <= S_ADDR; ctr <= 4'd0; end
                            else                    ctr <= ctr + 4'd1;
                        end
                        S_ADDR: begin
                            if (ctr == 4'd5) begin state <= S_MODE; ctr <= 4'd0; end
                            else                    ctr <= ctr + 4'd1;
                        end
                        S_MODE: begin
                            if (ctr == 4'd1) begin state <= S_DUMMY; ctr <= 4'd0; end
                            else                    ctr <= ctr + 4'd1;
                        end
                        S_DUMMY: begin
                            if (ctr == 4'd3) begin state <= S_READ_H; ctr <= 4'd0; end
                            else                    ctr <= ctr + 4'd1;
                        end
                        S_READ_H: begin
                            rx_word[15:8] <= {rx_word[11:8], qspi_d_in};
                            if (ctr == 4'd1) begin state <= S_READ_L; ctr <= 4'd0; end
                            else                    ctr <= ctr + 4'd1;
                        end
                        S_READ_L: begin
                            if (ctr == 4'd1) begin
                                ir_out      <= {rx_word[15:8], rx_word[3:0], qspi_d_in};
                                fetch_valid <= 1'b1;
                                heartbeat   <= ~heartbeat;
                                state       <= halted ? S_IDLE : S_READ_H;
                                ctr         <= 4'd0;
                            end else begin
                                rx_word[7:0] <= {rx_word[3:0], qspi_d_in};
                                ctr <= ctr + 4'd1;
                            end
                        end
                        default: ;
                    endcase
                end
            end
        end
    end

endmodule

`default_nettype wire
