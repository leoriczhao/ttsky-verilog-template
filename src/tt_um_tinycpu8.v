// tt_um_tinycpu8.v — Top-level TinyTapeout wrapper for TinyCPU-8 (SPEC v1.1 §6).
//
// Wires together: qspi_fetch + decoder + regfile + alu + pc + flags + IO latches.
//
// Pin mapping (SPEC §5):
//   ui_in  : read by IN instruction
//   uo_out : OUT-latched, overridden to PC[7:0] during BRK halt (§3.8)
//   uio[0] : qspi_cs_n (OUT)
//   uio[1] : qspi_sck  (OUT)
//   uio[5:2]: qspi_d3..d0 (bidir, driven by fetch FSM's qspi_d_oe)
//   uio[6] : brk_led  (= halted latch)
//   uio[7] : heartbeat (toggles on each successful fetch)

`default_nettype none

module tt_um_tinycpu8 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // Silence "unused" on `ena` — TinyTapeout keeps it high when project selected.
    wire _unused = ena;

    // ── Inter-module wires ───────────────────────────────────────────────
    wire [15:0] ir;
    wire        fetch_valid;

    wire [11:0] pc_val;
    wire [11:0] pc_plus_1;
    wire        pc_redirect;
    wire        pc_halted;

    wire [2:0]  rs1_addr, rs2_addr, rd_addr;
    wire        reg_we_dec;
    wire [2:0]  alu_funct;
    wire        alu_b_use_imm;
    wire [2:0]  wb_sel;
    wire        flag_z_we_dec, flag_c_we_dec;
    wire [2:0]  pc_op;
    wire [2:0]  branch_cond;
    wire        io_out_we_dec;
    wire [7:0]  imm8;
    wire [8:0]  imm9;
    wire [8:0]  branch_offset;
    wire [11:0] target12;
    wire        is_brk;

    wire [7:0]  rs1_data, rs2_data;
    wire [7:0]  wb_data;

    wire [7:0]  alu_y;
    wire        alu_z;
    wire        alu_c_out;
    wire        alu_c_update;

    wire        z_flag, c_flag;

    wire        qspi_cs_n_w, qspi_sck_w, heartbeat_w;
    wire [3:0]  qspi_d_out_w, qspi_d_oe_w;

    // ── Fetch ────────────────────────────────────────────────────────────
    qspi_fetch u_fetch (
        .clk         (clk),
        .rst_n       (rst_n),
        .pc_addr     (pc_val),
        .redirect    (pc_redirect),
        .halted      (pc_halted),
        .ir_out      (ir),
        .fetch_valid (fetch_valid),
        .qspi_cs_n   (qspi_cs_n_w),
        .qspi_sck    (qspi_sck_w),
        .qspi_d_out  (qspi_d_out_w),
        .qspi_d_oe   (qspi_d_oe_w),
        .qspi_d_in   (uio_in[5:2]),
        .heartbeat   (heartbeat_w)
    );

    // ── Decoder ──────────────────────────────────────────────────────────
    decoder u_dec (
        .ir            (ir),
        .rs1_addr      (rs1_addr),
        .rs2_addr      (rs2_addr),
        .rd_addr       (rd_addr),
        .reg_we        (reg_we_dec),
        .alu_funct     (alu_funct),
        .alu_b_use_imm (alu_b_use_imm),
        .wb_sel        (wb_sel),
        .flag_z_we     (flag_z_we_dec),
        .flag_c_we     (flag_c_we_dec),
        .pc_op         (pc_op),
        .branch_cond   (branch_cond),
        .io_out_we     (io_out_we_dec),
        .imm8          (imm8),
        .imm9          (imm9),
        .branch_offset (branch_offset),
        .target12      (target12),
        .is_brk        (is_brk),
        .is_nop        (),
        .is_reserved   ()
    );

    // ── Regfile (gated by fetch_valid so writes only happen in EXECUTE) ──
    regfile u_rf (
        .clk      (clk),
        .rst_n    (rst_n),
        .rs1_addr (rs1_addr),
        .rs2_addr (rs2_addr),
        .rd_addr  (rd_addr),
        .we       (reg_we_dec & fetch_valid),
        .wd       (wb_data),
        .rs1_data (rs1_data),
        .rs2_data (rs2_data)
    );

    // ── ALU ──────────────────────────────────────────────────────────────
    wire [7:0] alu_b = alu_b_use_imm ? imm8 : rs2_data;
    alu u_alu (
        .a        (rs1_data),
        .b        (alu_b),
        .funct    (alu_funct),
        .y        (alu_y),
        .z        (alu_z),
        .c_out    (alu_c_out),
        .c_update (alu_c_update)
    );

    // ── Writeback mux ────────────────────────────────────────────────────
    //   0 = ALU,  1 = imm8,  2 = ui_in,  3 = uio_in,  4 = pc+1[7:0] (CALL)
    assign wb_data = (wb_sel == 3'd1) ? imm8
                   : (wb_sel == 3'd2) ? ui_in
                   : (wb_sel == 3'd3) ? uio_in
                   : (wb_sel == 3'd4) ? pc_plus_1[7:0]
                   :                    alu_y;

    // ── PC ───────────────────────────────────────────────────────────────
    pc u_pc (
        .clk           (clk),
        .rst_n         (rst_n),
        .advance       (fetch_valid),
        .pc_op         (pc_op),
        .branch_cond   (branch_cond),
        .branch_offset (branch_offset),
        .target12      (target12),
        .ret_source    (rs1_data),
        .z_flag        (z_flag),
        .c_flag        (c_flag),
        .pc_out        (pc_val),
        .pc_plus_1     (pc_plus_1),
        .redirect      (pc_redirect),
        .halted        (pc_halted)
    );

    // ── Flags ────────────────────────────────────────────────────────────
    flags u_fl (
        .clk          (clk),
        .rst_n        (rst_n),
        .advance      (fetch_valid),
        .flag_z_we    (flag_z_we_dec),
        .flag_c_we    (flag_c_we_dec),
        .alu_z        (alu_z),
        .alu_c        (alu_c_out),
        .alu_c_update (alu_c_update),
        .z_flag       (z_flag),
        .c_flag       (c_flag)
    );

    // ── uo_out latch ─────────────────────────────────────────────────────
    reg [7:0] uo_out_latch;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            uo_out_latch <= 8'h00;
        else if (fetch_valid && io_out_we_dec)
            uo_out_latch <= rs1_data;
    end

    // During BRK halt, uo_out shows PC[7:0] of the BRK itself (SPEC §3.8).
    assign uo_out = pc_halted ? pc_val[7:0] : uo_out_latch;

    // ── uio wiring (see §5) ─────────────────────────────────────────────
    assign uio_out[0]   = qspi_cs_n_w;
    assign uio_out[1]   = qspi_sck_w;
    assign uio_out[5:2] = qspi_d_out_w;
    assign uio_out[6]   = pc_halted;       // brk_led
    assign uio_out[7]   = heartbeat_w;

    assign uio_oe[0]    = 1'b1;
    assign uio_oe[1]    = 1'b1;
    assign uio_oe[5:2]  = qspi_d_oe_w;
    assign uio_oe[6]    = 1'b1;
    assign uio_oe[7]    = 1'b1;

endmodule

`default_nettype wire
