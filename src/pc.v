// pc.v — Jimu-8 program counter + branch/jump/call/ret resolver
//         (SPEC v1.1 §2.2, §3.4–§3.6, §3.8).
//
// 12-bit word-addressed PC. Advances only on EXECUTE cycles (`advance`==1).
// Once a BRK has been seen, the `halted` latch freezes the PC until reset.
//
// pc_op encoding (matches decoder.v):
//     0 : sequential        pc ← pc + 1
//     1 : branch (cond)     pc ← branch_taken ? pc+1+sext(offset) : pc+1
//     2 : jmp               pc ← target12
//     3 : call              pc ← target12   (regfile writes R5 hi nibble, R6 lo byte)
//     4 : ret               pc ← ret_source[11:0]   (SPEC v1.2 §3.6: full 12 bits)
//     5 : halt (BRK)        pc holds; `halted` becomes 1 permanently
//
// `redirect` is asserted the cycle PC moves non-sequentially — the fetch FSM
// uses this to squash the in-flight QSPI read and restart at the new PC
// (SPEC §2.4, §4.5).

`default_nettype none

module pc (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        advance,        // EXECUTE stage firing this cycle
    input  wire [2:0]  pc_op,
    input  wire [2:0]  branch_cond,
    input  wire [8:0]  branch_offset,  // signed
    input  wire [11:0] target12,
    input  wire [11:0] ret_source,     // v1.2: {R5[3:0], R6[7:0]} full 12-bit

    input  wire        z_flag,
    input  wire        c_flag,

    output reg  [11:0] pc_out,
    output wire [11:0] pc_plus_1,
    output wire        redirect,
    output reg         halted
);

    // ── Branch-condition evaluation (SPEC §3.5) ─────────────────────────
    wire branch_flag  = branch_cond[1] ? c_flag : z_flag;
    wire branch_taken = (branch_cond == 3'b100)
                      | (~branch_cond[2] & (branch_flag ^ branch_cond[0]));

    // ── Next-PC computation ─────────────────────────────────────────────
    assign pc_plus_1 = pc_out + 12'd1;
    wire [11:0] sext_off  = {{3{branch_offset[8]}}, branch_offset};
    wire [11:0] br_target = pc_plus_1 + sext_off;

    wire op_is_branch    = (pc_op == 3'd1);
    wire op_is_jump_call = (pc_op[2:1] == 2'b01);
    wire op_is_ret       = (pc_op == 3'd4);
    wire nonseq          = (op_is_branch & branch_taken)
                         | op_is_jump_call
                         | op_is_ret;

    reg [11:0] next_pc;
    always @(*) begin
        next_pc = pc_plus_1;
        case (pc_op)
            3'd0: next_pc = pc_plus_1;
            3'd1: begin
                if (branch_taken)
                    next_pc = br_target;
            end
            3'd2,
            3'd3: begin
                next_pc = target12;
            end
            3'd4: begin
                next_pc = ret_source;            // v1.2: full 12-bit
            end
            3'd5: begin
                next_pc = pc_out;            // hold
            end
            default: next_pc = pc_plus_1;
        endcase
    end

    // Redirect is only meaningful on an EXECUTE cycle and when not already halted.
    assign redirect = advance & nonseq & ~halted;

    // ── PC / halted register ────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc_out <= 12'h000;
            halted <= 1'b0;
        end else if (advance && !halted) begin
            pc_out <= next_pc;
            if (pc_op == 3'd5) halted <= 1'b1;
        end
    end

endmodule

`default_nettype wire
