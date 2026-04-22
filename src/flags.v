// flags.v — Jimu-8 Z / C flag register (SPEC v1.1 §2.2, §3.3).
//
// Two DFFs. Update rules (combined outside by decoder + ALU):
//   Z <= flag_z_we  ? alu_z : Z                     (gated by `advance`)
//   C <= (flag_c_we & alu_c_update) ? alu_c : C     (gated by `advance`)
//
// `advance` lets the pipeline stall fetch without clobbering flags.
//
// `flag_z_we` / `flag_c_we` encode the decoder's "this instruction class may
// update Z / C" permission. The ALU's `alu_c_update` adds the operand-level
// veto for shift-by-0 (SPEC §3.3 — SHL/SHR leave C unchanged when shamt=0).

`default_nettype none

module flags (
    input  wire clk,
    input  wire rst_n,

    input  wire advance,          // EXECUTE stage firing
    input  wire flag_z_we,        // decoder: class-level Z-update permission
    input  wire flag_c_we,        // decoder: class-level C-update permission
    input  wire alu_z,
    input  wire alu_c,
    input  wire alu_c_update,     // ALU: operand-level C-update permission

    output reg  z_flag,
    output reg  c_flag
);

    wire z_we = advance & flag_z_we;
    wire c_we = advance & flag_c_we & alu_c_update;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            z_flag <= 1'b0;
            c_flag <= 1'b0;
        end else begin
            if (z_we) z_flag <= alu_z;
            if (c_we) c_flag <= alu_c;
        end
    end

endmodule

`default_nettype wire
