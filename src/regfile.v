// regfile.v — Jimu-8 register file (SPEC v1.1 §2.2).
//
// 7 general-purpose 8-bit registers R0..R6. R7 is hard-wired to 0x00:
// reads return 0; writes are silently discarded.
//
// Timing: async read, synchronous write on posedge clk.
// Same-cycle "read Rs, write Rd" reads the pre-write value (SPEC §2.4:
// "EXECUTE completes writeback in the same cycle as it reads", meaning the
// read sees the old Q and the new value is latched at the clock edge).

`default_nettype none

module regfile (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [2:0] rs1_addr,
    input  wire [2:0] rs2_addr,
    input  wire [2:0] rd_addr,
    input  wire       we,
    input  wire [7:0] wd,

    output wire [7:0] rs1_data,
    output wire [7:0] rs2_data
);

    reg [7:0] regs [0:6];
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 7; i = i + 1) regs[i] <= 8'h00;
        end else if (we && (rd_addr != 3'd7)) begin
            regs[rd_addr] <= wd;
        end
    end

    // R7 reads as 0; otherwise forward DFF output directly.
    assign rs1_data = (rs1_addr == 3'd7) ? 8'h00 : regs[rs1_addr];
    assign rs2_data = (rs2_addr == 3'd7) ? 8'h00 : regs[rs2_addr];

endmodule

`default_nettype wire
