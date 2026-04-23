// decoder.v — Jimu-8 instruction decoder (SPEC v1.1 §3).
//
// Pure combinational. Consumes the 16-bit IR; emits control signals plus the
// raw immediates/target for downstream consumers (ALU, PC, regfile, IO latch).
//
// Writeback source select (`wb_sel`):
//     0 : ALU result          (R-type, ADDI, ANDI)
//     1 : imm8                (MOVI)
//     2 : ui_in               (IN)
//     3 : uio_in              (UIO_IN)
//     4 : pc_plus_1[7:0]      (CALL: writes R6)
//
// PC operation (`pc_op`):
//     0 : sequential          (NOP, reserved, R-type, I-type, MOVI, IO)
//     1 : branch (conditional)  [B-type]
//     2 : jmp (unconditional)
//     3 : call (R6 <= PC+1[7:0]; PC <= target12)
//     4 : ret (PC <= {4'b0, R6})
//     5 : halt (BRK)
//
// Flag update gating: `flag_z_we` / `flag_c_we` carry instruction-class
// permission to write Z / C. Downstream the flags module AND's `flag_c_we`
// with the ALU's `c_update` (to honour "C unchanged on shift-by-0" per SPEC
// §3.3). Z is gated by `flag_z_we` only (the ALU always produces a valid Z).
//
// Reserved opcodes (1001..1101) decode as NOP-equivalent: no regwrite, no
// flag-write, sequential PC.
// Reserved IO subops (011..111 = UIO_OUT/UIO_DIR/101..111) also NOP: no reg
// effect, no uo_out latch.  UIO_OUT and UIO_DIR are intentionally NOPs on
// this silicon (SPEC §3.7 / §5).
// Reserved branch conds (101..111) are not filtered here; the PC module
// implements "never" for those cond values.

`default_nettype none

module decoder (
    input  wire [15:0] ir,

    // Regfile ports
    output wire [2:0]  rs1_addr,
    output wire [2:0]  rs2_addr,
    output wire [2:0]  rd_addr,
    output wire        reg_we,

    // ALU controls
    output wire [2:0]  alu_funct,
    output wire        alu_b_use_imm,

    // Writeback
    output wire [2:0]  wb_sel,

    // Flag class-level permissions
    output wire        flag_z_we,
    output wire        flag_c_we,

    // PC control
    output wire [2:0]  pc_op,
    output wire [2:0]  branch_cond,

    // IO side-effect latches
    output wire        io_out_we,     // uo_out_latch  <= rs1_data

    // Immediates / targets (consumers pick what they need)
    output wire [7:0]  imm8,          // ir[7:0]   — ADDI low / ANDI / MOVI
    output wire [8:0]  branch_offset, // ir[8:0]   — B-type signed
    output wire [11:0] target12,      // ir[11:0]  — J-type

    // Memory-op flags (v1.2) — only the ones the top actually consumes.
    // `is_ret` / `is_brk` / `is_nop` / `is_reserved` used to be exposed too
    // but nothing read them; the PC module handles RET/BRK via pc_op, and
    // reserved-opcode NOP semantics need no explicit flag. Removed to kill
    // Verilator UNUSEDSIGNAL / PINCONNECTEMPTY warnings. `imm9` was also
    // dropped for the same reason (imm8 + branch_offset cover all needs).
    output wire        is_load,       // opcode 1001 — PSRAM byte load
    output wire        is_store,      // opcode 1010 — PSRAM byte store
    output wire        is_call        // exposed for CALL pending-R5 logic
);

    // ── Field extraction ─────────────────────────────────────────────────
    wire [3:0] opcode    = ir[15:12];
    wire [2:0] f_rd      = ir[11:9];
    wire [2:0] f_rs1     = ir[8:6];
    wire [2:0] f_rs2     = ir[5:3];
    wire [2:0] f_funct   = ir[2:0];
    wire [2:0] f_iosub   = ir[8:6];
    wire [2:0] f_bcond   = ir[11:9];

    // ── Opcode classes ───────────────────────────────────────────────────
    wire is_rtype  = (opcode == 4'h0);
    wire is_addi   = (opcode == 4'h1);
    wire is_andi   = (opcode == 4'h2);
    wire is_itype  = is_addi | is_andi;
    wire is_br     = (opcode == 4'h3);
    wire is_jmp    = (opcode == 4'h4);
    wire is_call_w = (opcode == 4'h5);
    wire is_ret_w  = (opcode == 4'h6);
    wire is_io     = (opcode == 4'h7);
    wire is_movi   = (opcode == 4'h8);
    wire is_load_w  = (opcode == 4'h9);    // v1.2: LOAD
    wire is_store_w = (opcode == 4'hA);    // v1.2: STORE
    wire is_mem    = is_load_w | is_store_w;
    wire is_brkop  = (opcode == 4'hE);
    // NOP (4'hF) and reserved (4'hB..4'hD) decode as "do nothing" by falling
    // through every mux below — no dedicated flag is needed.

    // Expose these classes for the top-level arbiter (CALL pending-R5,
    // LOAD/STORE stall + deferred LOAD writeback).
    assign is_load  = is_load_w;
    assign is_store = is_store_w;
    assign is_call  = is_call_w;

    // ── IO subop classification ──────────────────────────────────────────
    wire io_is_in     = is_io & (f_iosub == 3'b000);
    wire io_is_out    = is_io & (f_iosub == 3'b001);
    wire io_is_uio_in = is_io & (f_iosub == 3'b010);
    // 3'b011..111 all NOP (UIO_OUT, UIO_DIR, reserved)

    // ── Register addresses ───────────────────────────────────────────────
    //
    // v1.2 changes:
    //   RET reads both R6 (low 8 bits) and R5 (high 4 bits of PC+1).
    //   LOAD/STORE read Rhi and Rlo (ir[8:6], ir[5:3]) for the 16-bit byte
    //   address; STORE additionally needs Rs (= ir[11:9]) for the data
    //   byte, which the top-level FSM reads by re-driving rs1_addr during
    //   the PSRAM transaction (out-of-EXECUTE; not the decoder's concern).
    //
    assign rs1_addr = is_rtype ? f_rs1
                    : is_itype ? f_rd                     // 2-op: Rs1 = Rd
                    : is_io    ? f_rd
                    : is_ret_w ? 3'd6                     // RET rs1 = R6 (lo)
                    : is_mem   ? f_rs1                    // LOAD/STORE: Rhi
                    : 3'd0;
    assign rs2_addr = is_rtype ? f_rs2
                    : is_ret_w ? 3'd5                     // RET rs2 = R5 (hi)
                    : is_mem   ? f_rs2                    // LOAD/STORE: Rlo
                    : 3'd0;
    assign rd_addr  = is_call                        ? 3'd6    // CALL link lo
                    : (is_rtype | is_itype | is_movi) ? f_rd
                    : (io_is_in | io_is_uio_in)       ? f_rd
                    : is_load_w                        ? f_rd   // LOAD dest
                    : 3'd0;

    // Regfile write enable. (R7 writes are discarded inside regfile.)
    // NOTE: LOAD's regfile write is DEFERRED to when PSRAM data arrives —
    // handled by the top-level arbiter, not here. STORE never writes regs.
    assign reg_we = is_rtype | is_itype | is_movi | is_call
                  | io_is_in | io_is_uio_in;

    // ── ALU control ──────────────────────────────────────────────────────
    // R-type: funct = ir[2:0]. I-type: forced ADD/AND.
    assign alu_funct = is_rtype ? f_funct
                     : is_addi  ? 3'b000    // ADD
                     : is_andi  ? 3'b010    // AND
                     :            3'b000;
    assign alu_b_use_imm = is_itype;         // else use rs2_data

    // ── Writeback mux select ─────────────────────────────────────────────
    assign wb_sel = is_movi      ? 3'd1
                  : io_is_in     ? 3'd2
                  : io_is_uio_in ? 3'd3
                  : is_call      ? 3'd4
                  :                3'd0;

    // ── Flag update permissions (class-level) ────────────────────────────
    //   R-type : Z always; C handled by ALU c_update
    //   ADDI   : Z + C
    //   ANDI   : Z only
    //   others : neither
    assign flag_z_we = is_rtype | is_addi | is_andi;
    assign flag_c_we = is_rtype | is_addi;

    // ── PC control ───────────────────────────────────────────────────────
    assign pc_op = is_br     ? 3'd1
                 : is_jmp    ? 3'd2
                 : is_call   ? 3'd3
                 : is_ret_w  ? 3'd4
                 : is_brkop  ? 3'd5
                 :             3'd0;
    assign branch_cond = f_bcond;

    // ── IO side-effect ──────────────────────────────────────────────────
    assign io_out_we = io_is_out;

    // ── Immediates & targets ─────────────────────────────────────────────
    assign imm8          = ir[7:0];
    assign branch_offset = ir[8:0];
    assign target12      = ir[11:0];

endmodule

`default_nettype wire
