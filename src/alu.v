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
// SHL carry derivation: shift-by-1 → a[7]; shift-by-2 → a[6]; … shift-by-7 → a[1].
// SHR carry derivation: shift-by-1 → a[0]; shift-by-2 → a[1]; … shift-by-7 → a[6].
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

    wire [2:0]  shamt    = b[2:0];
    wire        addsub_sub = funct[0];
    wire [7:0]  addsub_b   = b ^ {8{addsub_sub}};
    wire [8:0]  addsub9    = {1'b0, a} + {1'b0, addsub_b}
                           + {8'b0, addsub_sub};

    wire        shift_left    = funct[0];
    wire [7:0]  a_rev         = {a[0], a[1], a[2], a[3],
                                 a[4], a[5], a[6], a[7]};
    wire [7:0]  shift_in      = shift_left ? a_rev : a;
    wire [7:0]  shift_right_y = shift_in >> shamt;
    wire [7:0]  shift_y       = shift_left ? {shift_right_y[0], shift_right_y[1],
                                              shift_right_y[2], shift_right_y[3],
                                              shift_right_y[4], shift_right_y[5],
                                              shift_right_y[6], shift_right_y[7]}
                                           : shift_right_y;
    wire        shift_c       = (shamt == 3'd0) ? 1'b0
                                                : shift_in[shamt - 3'd1];

    always @(*) begin
        c_out    = 1'b0;
        c_update = 1'b0;
        case (funct)
            3'b000,
            3'b001: begin                              // ADD / SUB
                y        = addsub9[7:0];
                c_out    = addsub9[8];
                c_update = 1'b1;
            end
            3'b010:   y = a & b;                       // AND
            3'b011:   y = a | b;                       // OR
            3'b100:   y = a ^ b;                       // XOR
            3'b101,
            3'b110: begin                              // SHL / SHR (logical)
                y        = shift_y;
                c_out    = shift_c;
                c_update = (shamt != 3'd0);
            end
            3'b111:   y = ~a;                          // NOT
            default:  y = 8'h00;
        endcase
    end

    assign z = (y == 8'h00);

endmodule

`default_nettype wire
