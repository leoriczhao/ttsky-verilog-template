// alu.v — Jimu-8 arithmetic / logic unit (SPEC v1.1 §3.3).
//
// Inputs:  a, b (8-bit operands)     funct[2:0] selects operation.
// Outputs: y (8-bit result)
//          z              = (y == 0)        — Z flag candidate (always valid)
//          c_out          = new carry value — only meaningful when c_update=1
//          c_update       = 1 if THIS operation/operand actually wants to
//                           write C. Combined with a decoder-level
//                           "instruction-class updates C" signal outside.
//
// Flag-update rules (matching SPEC §3.3):
//   ADD / SUB              : always update C
//   AND / OR / XOR / NOT   : C unchanged                   → c_update=0
//   SHL / SHR with shamt=0 : C unchanged                   → c_update=0
//   SHL / SHR with shamt!=0: C = last bit shifted out      → c_update=1
//
// SHL carry derivation:    shl_wide = {8'b0,a} << shamt;  carry = shl_wide[8]
//   Shift-by-1 → a[7];  shift-by-2 → a[6]; … shift-by-7 → a[1].
// SHR carry derivation:    shr_wide = {a,8'b0} >> shamt;  carry = shr_wide[7]
//   Shift-by-1 → a[0];  shift-by-2 → a[1]; … shift-by-7 → a[6].
//
// Z is always recomputed as (y == 0). Whether Z is written to the flag register
// is gated outside (e.g., MOVI doesn't update Z; decoder handles that).

`default_nettype none

module alu (
    input  wire [7:0] a,
    input  wire [7:0] b,
    input  wire [2:0] funct,

    output reg  [7:0] y,
    output wire       z,
    output reg        c_out,
    output reg        c_update
);

    wire [8:0]  add9     = {1'b0, a} + {1'b0, b};
    wire [8:0]  sub9     = {1'b0, a} - {1'b0, b};
    wire [2:0]  shamt    = b[2:0];
    wire [15:0] shl_wide = {8'b0, a} << shamt;
    wire [15:0] shr_wide = {a, 8'b0} >> shamt;

    always @(*) begin
        c_out    = 1'b0;
        c_update = 1'b0;
        case (funct)
            3'b000: begin                              // ADD
                y        = add9[7:0];
                c_out    = add9[8];
                c_update = 1'b1;
            end
            3'b001: begin                              // SUB (C=1 if no borrow)
                y        = sub9[7:0];
                c_out    = ~sub9[8];
                c_update = 1'b1;
            end
            3'b010:   y = a & b;                       // AND
            3'b011:   y = a | b;                       // OR
            3'b100:   y = a ^ b;                       // XOR
            3'b101: begin                              // SHL
                y        = shl_wide[7:0];
                c_out    = shl_wide[8];
                c_update = (shamt != 3'd0);
            end
            3'b110: begin                              // SHR (logical)
                y        = shr_wide[15:8];
                c_out    = shr_wide[7];
                c_update = (shamt != 3'd0);
            end
            3'b111:   y = ~a;                          // NOT
            default:  y = 8'h00;
        endcase
    end

    assign z = (y == 8'h00);

endmodule

`default_nettype wire
