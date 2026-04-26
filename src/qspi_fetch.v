// qspi_fetch.v — Jimu-8 QSPI arbiter (SPEC v1.2 §4).
//
// Drives a shared 4-wire QSPI bus to TWO chips on the TinyTapeout QSPI Pmod:
//
//   * Flash (W25Q128-class), CS on uio[0]    — instruction fetch path
//   * PSRAM (APS6404L-class), CS on uio[6]   — LOAD / STORE data path
//
// Pin mapping (SPEC §5, v1.2):
//   uio[0] = flash_cs_n   (out)      uio[3] = qspi_sck   (out)
//   uio[1] = d0 / MOSI    (bidir)    uio[4] = d2         (bidir)
//   uio[2] = d1 / MISO    (bidir)    uio[5] = d3         (bidir)
//   uio[6] = psram_cs_n   (out)      uio[7] = ram_b_disable (hardwired 1 at top)
//
// This module outputs `qspi_d_out[3:0]` and `qspi_d_oe[3:0]` as 4-bit
// nibble-aligned signals. The TOP module wires them to uio[1,2,4,5].
//
// ── Arbitration priority (high → low) ────────────────────────────────────
//   1.  reset or redirect or halted  → flash fetch aborts to IDLE
//   2.  mem_op_start (LOAD/STORE)    → flash aborts to IDLE, PSRAM runs
//   3.  otherwise                    → continuous flash fetch
//
// Both CS lines must never be low simultaneously (TB asserts this).
//
// ── SPI timing ────────────────────────────────────────────────────────────
//   * sck_r is a registered output toggling every clk while any FSM is
//     active. Same approach as v1.1 — avoids combinational-gating glitches.
//   * Flash phase timing matches v1.1 (see qspi_fetch v1.1 doc header).
//
// ── PSRAM timing (APS6404L 0xEB / 0x38) ───────────────────────────────────
//   Fast Quad Read (0xEB):
//     CMD   = 8 SPI cycles (x1 on D0)       = 16 CPU cycles
//     ADDR  = 6 SPI cycles (x4)             = 12 CPU cycles
//     DUMMY = 6 SPI cycles                  = 12 CPU cycles
//     DATA  = 2 SPI cycles (x4, 1 byte)     =  4 CPU cycles
//   Quad Write (0x38):
//     CMD/ADDR as above; no dummy; DATA 2 cycles.
//   S_PS_DATA (ctr==1) transitions directly to S_CMD: mem_op_done pulses,
//   the top's PC advances on that same edge, and S_ADDR (combinational
//   shift of `pc_addr`) then drives the new fetch address.
//
// ── Address path (no internal copy) ───────────────────────────────────────
//   Neither the flash nor the PSRAM address is registered inside this
//   module. S_ADDR shifts directly from the `pc_addr` input (stable across
//   the CMD+ADDR window — PC only moves on fetch_valid / mem_op_done).
//   S_PS_ADDR shifts directly from `mem_addr`, held stable by the top-level
//   regfile/address muxes for the PSRAM address window. Saves local address
//   copies in the QSPI FSM.

`default_nettype none

module qspi_fetch (
    input  wire        clk,
    input  wire        rst_n,

    // ── PC / EXECUTE interface ─────────────────────────────────────────
    input  wire [11:0] pc_addr,
    input  wire        redirect,        // branch flush
    input  wire        halted,          // BRK

    // ── Memory-op interface (v1.2) ─────────────────────────────────────
    input  wire        mem_op_start,    // 1-cycle pulse: start LOAD/STORE
    input  wire        mem_is_store,    // 1 = STORE, 0 = LOAD
    input  wire [15:0] mem_addr,        // {Rhi, Rlo} byte address
    input  wire [7:0]  mem_wdata,       // STORE data, stable through STORE txn

    output wire [7:0]  mem_rdata,       // LOAD result, valid with mem_op_done
    output reg         mem_op_done,     // 1-cycle pulse at memory-op completion

    // ── Fetch result ───────────────────────────────────────────────────
    output reg  [15:0] ir_out,
    output reg         fetch_valid,

    // ── QSPI pins (top remaps to uio[]) ────────────────────────────────
    output wire        flash_cs_n,
    output wire        psram_cs_n,
    output wire        qspi_sck,
    output wire [3:0]  qspi_d_out,
    output wire [3:0]  qspi_d_oe,
    input  wire [3:0]  qspi_d_in
);

    // ── State encoding ────────────────────────────────────────────────
    localparam [3:0]
        S_IDLE     = 4'd0,    // all CS high
        S_CMD      = 4'd1,    // flash: 0xEB on D0
        S_ADDR     = 4'd2,    // flash: 24-bit addr, x4
        S_MODE     = 4'd3,    // flash: 0x00, x4
        S_DUMMY    = 4'd4,    // flash: 4 dummy cycles
        S_READ_H   = 4'd5,
        S_READ_L   = 4'd6,
        S_PS_CMD   = 4'd8,    // PSRAM: cmd byte on D0, 8 SPI cycles
        S_PS_ADDR  = 4'd9,    // PSRAM: 24-bit addr, x4, 6 SPI cycles
        S_PS_DUMMY = 4'd10,   // PSRAM: 6 dummy (read only)
        S_PS_DATA  = 4'd11;   // PSRAM: 1 byte, x4, 2 SPI cycles
        // bit[3]=1 discriminates PSRAM states (S_PS_*) from flash states.

    localparam [7:0] FLASH_CMD_BYTE  = 8'hEB;
    localparam [7:0] FLASH_MODE_BYTE = 8'h00;
    localparam [7:0] PS_CMD_READ     = 8'hEB;
    localparam [7:0] PS_CMD_WRITE    = 8'h38;

    reg [3:0]  state;
    reg        sck_r;
    reg [3:0]  ctr;

    // Flash datapath: only the rx shift register needs storage. The 24-bit
    // send address is shifted out combinationally from `pc_addr`.
    reg [15:0] flash_rx_word;

    // PSRAM datapath state
    reg [7:0]  ps_rdata;        // byte read back; after S_PS_DATA finishes
                                // this holds the full byte — used directly
                                // as mem_rdata, saving a separate 8-DFF latch.

    // mem_is_store is decoded from the held IR in top, so it stays stable for
    // the full PSRAM transaction without a local copy.
    wire [7:0] ps_cmd_byte = mem_is_store ? PS_CMD_WRITE : PS_CMD_READ;

    // ── Combinational outputs ────────────────────────────────────────
    wire flash_active = (state != S_IDLE) && (state[3] == 1'b0);
    wire psram_active =  state[3] == 1'b1;
    assign flash_cs_n = ~flash_active;
    assign psram_cs_n = ~psram_active;
    assign qspi_sck   = sck_r;

    // Flash ADDR nibble (24-bit big-endian address = {11'b0, pc_addr, 1'b0}):
    //   ctr 0,1 → high zeros; ctr 2..5 → the 12 pc_addr bits + final 0.
    reg [3:0] flash_addr_nib;
    always @(*) begin
        case (ctr[2:0])
            3'd2:    flash_addr_nib = {3'b000, pc_addr[11]};
            3'd3:    flash_addr_nib =  pc_addr[10:7];
            3'd4:    flash_addr_nib =  pc_addr[6:3];
            3'd5:    flash_addr_nib = {pc_addr[2:0], 1'b0};
            default: flash_addr_nib = 4'h0;   // ctr 0,1 (and fill)
        endcase
    end

    // PSRAM ADDR nibble (24-bit big-endian = {8'h00, mem_addr}):
    //   ctr 0,1 → high zeros; ctr 2..5 → the 16 mem_addr bits.
    reg [3:0] ps_addr_nib;
    always @(*) begin
        case (ctr[2:0])
            3'd2:    ps_addr_nib = mem_addr[15:12];
            3'd3:    ps_addr_nib = mem_addr[11:8];
            3'd4:    ps_addr_nib = mem_addr[7:4];
            3'd5:    ps_addr_nib = mem_addr[3:0];
            default: ps_addr_nib = 4'h0;
        endcase
    end

    // CMD-byte (1-bit) and nibble (4-bit) extractors used below. Using
    // explicit bit-select / concat form instead of `>>` + `& mask` keeps
    // RHS width == LHS width so Verilator doesn't warn WIDTHTRUNC.
    wire [2:0] cmd_bit_idx = 3'd7 - ctr[2:0];    // 7..0 as ctr goes 0..7
    wire       flash_cmd_bit = FLASH_CMD_BYTE[cmd_bit_idx];
    wire       ps_cmd_bit    = ps_cmd_byte[cmd_bit_idx];
    // MODE byte is 8'h00, always zero — kept as a wire so yosys prunes it.
    wire [3:0] flash_mode_nib = ctr[0] ? FLASH_MODE_BYTE[3:0] : FLASH_MODE_BYTE[7:4];
    // STORE-DATA nibble: ctr=0 → high nibble, ctr=1 → low nibble.
    wire [3:0] ps_wdata_nib   = ctr[0] ? mem_wdata[3:0] : mem_wdata[7:4];

    reg [3:0] d_oe_c, d_out_c;
    always @(*) begin
        d_oe_c  = 4'b0000;
        d_out_c = 4'b0000;
        case (state)
            S_CMD:  begin
                d_oe_c  = 4'b0001;
                d_out_c = {3'b0, flash_cmd_bit};
            end
            S_ADDR: begin
                d_oe_c  = 4'b1111;
                d_out_c = flash_addr_nib;
            end
            S_MODE: begin
                d_oe_c  = 4'b1111;
                d_out_c = flash_mode_nib;
            end
            S_PS_CMD: begin
                d_oe_c  = 4'b0001;
                d_out_c = {3'b0, ps_cmd_bit};
            end
            S_PS_ADDR: begin
                d_oe_c  = 4'b1111;
                d_out_c = ps_addr_nib;
            end
            S_PS_DATA: begin
                // In STORE phase the master drives; in LOAD phase the slave does.
                if (mem_is_store) begin
                    d_oe_c  = 4'b1111;
                    d_out_c = ps_wdata_nib;
                end
            end
            default: ;
        endcase
    end
    assign qspi_d_out = d_out_c;
    assign qspi_d_oe  = d_oe_c;

    // LOAD result comes out of ps_rdata — by the time `mem_op_done` pulses
    // (S_PS_DATA, ctr==1), ps_rdata has been shifted with the second nibble
    // and holds the full byte. Saves an 8-DFF mem_rdata latch.
    assign mem_rdata = ps_rdata;


    // ── FSM ──────────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            sck_r          <= 1'b0;
            ctr            <= 4'd0;
            flash_rx_word  <= 16'h0;
            ps_rdata       <= 8'h00;
            ir_out         <= 16'hFFFF;
            fetch_valid    <= 1'b0;
            mem_op_done    <= 1'b0;
        end else begin
            fetch_valid <= 1'b0;
            mem_op_done <= 1'b0;

            // Priority 1: redirect or halted → abort everything back to IDLE.
            if (redirect || halted) begin
                state <= S_IDLE;
                sck_r <= 1'b0;
                ctr   <= 4'd0;
            end

            // Priority 2: mem_op_start → begin PSRAM transaction.
            //             Must come AFTER redirect/halted check so they win.
            else if (mem_op_start) begin
                state       <= S_PS_CMD;
                sck_r       <= 1'b0;
                ctr         <= 4'd0;
                // Address (mem_addr) is NOT latched here — S_PS_ADDR shifts
                // it combinationally from the input port.
            end

            // Priority 3: run whatever FSM path is active.
            else if (state == S_IDLE) begin
                sck_r <= 1'b0;
                if (!halted) begin
                    state <= S_CMD;
                    ctr   <= 4'd0;
                    // flash address is NOT latched — S_ADDR shifts it
                    // combinationally from `pc_addr` (stable through the
                    // CMD+ADDR window).
                end
            end
            else begin
                // Any active state: toggle SCK each clk.
                sck_r <= ~sck_r;

                if (sck_r == 1'b0) begin
                    // Rising-edge boundary: counter advance + input sample.
                    case (state)
                        // ── Flash path ──
                        S_CMD: begin
                            if (ctr == 4'd7) begin state <= S_ADDR; ctr <= 4'd0; end
                            else              ctr <= ctr + 4'd1;
                        end
                        S_ADDR: begin
                            if (ctr == 4'd5) begin state <= S_MODE;  ctr <= 4'd0; end
                            else              ctr <= ctr + 4'd1;
                        end
                        S_MODE: begin
                            if (ctr == 4'd1) begin state <= S_DUMMY; ctr <= 4'd0; end
                            else              ctr <= ctr + 4'd1;
                        end
                        S_DUMMY: begin
                            if (ctr == 4'd3) begin state <= S_READ_H; ctr <= 4'd0; end
                            else              ctr <= ctr + 4'd1;
                        end
                        S_READ_H: begin
                            flash_rx_word[15:8] <= {flash_rx_word[11:8], qspi_d_in};
                            if (ctr == 4'd1) begin state <= S_READ_L; ctr <= 4'd0; end
                            else              ctr <= ctr + 4'd1;
                        end
                        S_READ_L: begin
                            if (ctr == 4'd1) begin
                                ir_out      <= {flash_rx_word[15:8],
                                                flash_rx_word[3:0], qspi_d_in};
                                fetch_valid <= 1'b1;
                                state       <= halted ? S_IDLE : S_READ_H;
                                ctr         <= 4'd0;
                            end else begin
                                // Only [3:0] is read below (at ctr==1 commit);
                                // we used to do a full [7:0] shift, but [7:4]
                                // was never consumed — drop it.
                                flash_rx_word[3:0] <= qspi_d_in;
                                ctr <= ctr + 4'd1;
                            end
                        end

                        // ── PSRAM path ──
                        S_PS_CMD: begin
                            if (ctr == 4'd7) begin state <= S_PS_ADDR; ctr <= 4'd0; end
                            else              ctr <= ctr + 4'd1;
                        end
                        S_PS_ADDR: begin
                            if (ctr == 4'd5) begin
                                if (mem_is_store) begin
                                    state <= S_PS_DATA;     // writes skip DUMMY
                                    ctr   <= 4'd0;
                                end else begin
                                    state <= S_PS_DUMMY;
                                    ctr   <= 4'd0;
                                end
                            end else
                                ctr <= ctr + 4'd1;
                        end
                        S_PS_DUMMY: begin
                            // 6 dummy SPI cycles for Fast-Quad-Read
                            if (ctr == 4'd5) begin state <= S_PS_DATA; ctr <= 4'd0; end
                            else              ctr <= ctr + 4'd1;
                        end
                        S_PS_DATA: begin
                            if (!mem_is_store) begin
                                // LOAD: shift in 4 bits per SPI cycle; after
                                // ctr==1 ps_rdata holds the full byte and
                                // is exported as mem_rdata (no extra latch).
                                ps_rdata <= {ps_rdata[3:0], qspi_d_in};
                            end
                            if (ctr == 4'd1) begin
                                // Retire: pulse mem_op_done (top advances PC
                                // on this same edge) and transition straight
                                // to S_CMD. Next-cycle S_ADDR will shift the
                                // post-advance pc_addr. No S_PS_DONE buffer.
                                state       <= halted ? S_IDLE : S_CMD;
                                mem_op_done <= 1'b1;
                                ctr         <= 4'd0;
                            end else
                                ctr <= ctr + 4'd1;
                        end

                        default: ;
                    endcase
                end
            end
        end
    end

endmodule

`default_nettype wire
