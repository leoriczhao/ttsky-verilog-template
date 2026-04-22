// tt_um_tinycpu8.v — Top-level TinyTapeout wrapper for Jimu-8 v1.2.
//
// Glues together: qspi_fetch (flash+PSRAM arbiter) + decoder + regfile
// + alu + pc + flags + IO latches.
//
// Orchestrates three writeback-path priorities (all mutually exclusive in
// practice):
//   1. LOAD completion — FSM drives Rd ← mem_rdata at mem_op_done.
//   2. CALL pending R5 write — one cycle after a CALL EXECUTE, while the
//      flash FSM is restarting, we write R5 ← pc_plus_1[11:8].
//   3. Normal EXECUTE writeback — R-type / I-type / MOVI / IN / UIO_IN /
//      CALL's R6 link.
//
// Also time-shares the regfile rs1 read port during a STORE: during the
// EXECUTE cycle rs1=Rhi (for address), on the very next cycle we override
// rs1 to ir[11:9]=Rs to latch the byte being stored into the FSM.
//
// Pinout (SPEC §5, v1.2):
//   uio[0] = flash_cs_n     uio[3] = qspi_sck
//   uio[1] = d0 / MOSI      uio[4] = d2
//   uio[2] = d1 / MISO      uio[5] = d3
//   uio[6] = psram_cs_n     uio[7] = ram_b_disable (hardwired 1)

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
    wire _unused_ena = ena;

    // ── Inter-module wires ───────────────────────────────────────────
    wire [15:0] ir;
    wire        fetch_valid;

    wire [11:0] pc_val;
    wire [11:0] pc_plus_1;
    wire        pc_redirect;
    wire        pc_halted;

    wire [2:0]  dec_rs1_addr, dec_rs2_addr, dec_rd_addr;
    wire        dec_reg_we;
    wire [2:0]  alu_funct;
    wire        alu_b_use_imm;
    wire [2:0]  wb_sel;
    wire        flag_z_we_dec, flag_c_we_dec;
    wire [2:0]  pc_op;
    wire [2:0]  branch_cond;
    wire        io_out_we_dec;
    wire [7:0]  imm8;
    wire [8:0]  branch_offset;
    wire [11:0] target12;
    wire        is_load, is_store, is_call_w;
    // imm9, is_ret (full 9-bit imm and ret flag) / is_brk are exposed by
    // the decoder for debugging but not consumed here — leave the ports
    // dangling rather than carrying a dead wire.

    wire [7:0]  rs1_data, rs2_data;
    wire [7:0]  wb_data;

    wire [7:0]  alu_y;
    wire        alu_z;
    wire        alu_c_out;
    wire        alu_c_update;

    wire        z_flag, c_flag;

    // QSPI
    wire        flash_cs_n_w, psram_cs_n_w, qspi_sck_w;
    wire [3:0]  qspi_d_out_w, qspi_d_oe_w;
    wire [3:0]  qspi_d_in_w;

    // Memory-op interface to qspi_fetch
    reg         mem_op_latched;       // set in cycle after LOAD/STORE EXECUTE
    reg         mem_op_is_store;
    reg [15:0]  mem_op_addr_lat;
    wire [7:0]  mem_rdata;
    wire        mem_op_done;
    wire        mem_op_start = mem_op_latched;
    wire [7:0]  mem_wdata_now = rs1_data;   // during mem_op_latched, rs1 override gives Rs

    // Writeback arbitration
    wire        mem_load_complete = mem_op_done & ~mem_op_is_store;

    // CALL pending R5 deferred write
    reg         pending_r5_we;
    reg [3:0]   pending_r5_data;

    // ── Fetch / arbiter ──────────────────────────────────────────────
    qspi_fetch u_fetch (
        .clk          (clk),
        .rst_n        (rst_n),
        .pc_addr      (pc_val),
        .redirect     (pc_redirect),
        .halted       (pc_halted),

        .mem_op_start (mem_op_start),
        .mem_is_store (mem_op_is_store),
        .mem_addr     (mem_op_addr_lat),
        .mem_wdata    (mem_wdata_now),
        .mem_rdata    (mem_rdata),
        .mem_op_done  (mem_op_done),

        .ir_out       (ir),
        .fetch_valid  (fetch_valid),

        .flash_cs_n   (flash_cs_n_w),
        .psram_cs_n   (psram_cs_n_w),
        .qspi_sck     (qspi_sck_w),
        .qspi_d_out   (qspi_d_out_w),
        .qspi_d_oe    (qspi_d_oe_w),
        .qspi_d_in    (qspi_d_in_w)
    );

    // ── Decoder ──────────────────────────────────────────────────────
    decoder u_dec (
        .ir            (ir),
        .rs1_addr      (dec_rs1_addr),
        .rs2_addr      (dec_rs2_addr),
        .rd_addr       (dec_rd_addr),
        .reg_we        (dec_reg_we),
        .alu_funct     (alu_funct),
        .alu_b_use_imm (alu_b_use_imm),
        .wb_sel        (wb_sel),
        .flag_z_we     (flag_z_we_dec),
        .flag_c_we     (flag_c_we_dec),
        .pc_op         (pc_op),
        .branch_cond   (branch_cond),
        .io_out_we     (io_out_we_dec),
        .imm8          (imm8),
        .imm9          (),
        .branch_offset (branch_offset),
        .target12      (target12),
        .is_load       (is_load),
        .is_store      (is_store),
        .is_call       (is_call_w),
        .is_ret        (),
        .is_brk        (),
        .is_nop        (),
        .is_reserved   ()
    );

    // ── Regfile read-port arbiter ────────────────────────────────────
    // During the cycle AFTER a LOAD/STORE EXECUTE (mem_op_latched=1), we
    // override rs1_addr to the Rd field of the current IR so rs1_data
    // presents the Rs byte to be stored. decoder's rs1 is irrelevant that
    // cycle because fetch_valid=0 (no EXECUTE commit).
    wire [2:0] rs1_addr_final = mem_op_latched ? ir[11:9] : dec_rs1_addr;
    wire [2:0] rs2_addr_final = dec_rs2_addr;

    // Regfile write-port arbiter (priority: LOAD-complete > CALL-R5 > normal).
    wire       rf_we_final   = mem_load_complete | pending_r5_we
                             | (dec_reg_we & fetch_valid);
    wire [2:0] rf_waddr_final = mem_load_complete ? ir[11:9]
                              : pending_r5_we     ? 3'd5
                              : dec_rd_addr;
    wire [7:0] rf_wdata_final = mem_load_complete ? mem_rdata
                              : pending_r5_we     ? {4'b0000, pending_r5_data}
                              : wb_data;

    regfile u_rf (
        .clk      (clk),
        .rst_n    (rst_n),
        .rs1_addr (rs1_addr_final),
        .rs2_addr (rs2_addr_final),
        .rd_addr  (rf_waddr_final),
        .we       (rf_we_final),
        .wd       (rf_wdata_final),
        .rs1_data (rs1_data),
        .rs2_data (rs2_data)
    );

    // ── ALU ──────────────────────────────────────────────────────────
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

    // ── Writeback data mux (normal EXECUTE path) ─────────────────────
    //   0 = ALU,  1 = imm8,  2 = ui_in,  3 = uio_in,  4 = pc+1[7:0] (CALL)
    assign wb_data = (wb_sel == 3'd1) ? imm8
                   : (wb_sel == 3'd2) ? ui_in
                   : (wb_sel == 3'd3) ? uio_in
                   : (wb_sel == 3'd4) ? pc_plus_1[7:0]
                   :                    alu_y;

    // ── PC ───────────────────────────────────────────────────────────
    // RET now reconstructs full 12-bit PC from R5 (high nibble) + R6 (low byte).
    // For RET: rs1=R6 (low), rs2=R5 (high nibble in [3:0]).
    wire [11:0] ret_source_12 = {rs2_data[3:0], rs1_data[7:0]};

    // PC advance:
    //   * Normal EXECUTE cycles (fetch_valid), but SKIP LOAD/STORE — they
    //     take many cycles and complete via mem_op_done.
    //   * mem_op_done — retirement pulse for LOAD/STORE, advances PC by 1.
    //
    // Without this separation PC would double-advance on LOAD/STORE
    // (once at fetch_valid, again at mem_op_done) and skip the following
    // instruction.
    wire pc_advance = (fetch_valid & ~(is_load | is_store)) | mem_op_done;
    wire [2:0] pc_op_final = mem_op_done ? 3'd0 : pc_op;

    pc u_pc (
        .clk           (clk),
        .rst_n         (rst_n),
        .advance       (pc_advance),
        .pc_op         (pc_op_final),
        .branch_cond   (branch_cond),
        .branch_offset (branch_offset),
        .target12      (target12),
        .ret_source    (ret_source_12),
        .z_flag        (z_flag),
        .c_flag        (c_flag),
        .pc_out        (pc_val),
        .pc_plus_1     (pc_plus_1),
        .redirect      (pc_redirect),
        .halted        (pc_halted)
    );

    // ── Flags ────────────────────────────────────────────────────────
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

    // ── Memory-op latching ───────────────────────────────────────────
    // When fetch_valid for a LOAD or STORE, latch the address and set
    // mem_op_latched. The latched pulse is what triggers mem_op_start in
    // qspi_fetch on the NEXT cycle. The NEXT cycle also presents Rs on
    // rs1 so we can latch mem_wdata via the arbiter inside qspi_fetch.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_op_latched  <= 1'b0;
            mem_op_is_store <= 1'b0;
            mem_op_addr_lat <= 16'h0;
        end else begin
            if (fetch_valid & (is_load | is_store)) begin
                mem_op_latched  <= 1'b1;
                mem_op_is_store <= is_store;
                mem_op_addr_lat <= {rs1_data, rs2_data};
            end else if (mem_op_latched) begin
                mem_op_latched <= 1'b0;
                // mem_op_is_store stays valid until mem_op_done so writeback
                // arbiter can distinguish LOAD vs STORE retirement.
            end
        end
    end

    // ── CALL pending-R5 deferred write ──────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_r5_we   <= 1'b0;
            pending_r5_data <= 4'h0;
        end else begin
            if (fetch_valid & is_call_w) begin
                pending_r5_we   <= 1'b1;
                pending_r5_data <= pc_plus_1[11:8];
            end else if (pending_r5_we) begin
                pending_r5_we <= 1'b0;     // auto-clear after the write commits
            end
        end
    end

    // ── uo_out latch + BRK override ─────────────────────────────────
    reg [7:0] uo_out_latch;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            uo_out_latch <= 8'h00;
        else if (fetch_valid && io_out_we_dec)
            uo_out_latch <= rs1_data;
    end

    assign uo_out = pc_halted ? pc_val[7:0] : uo_out_latch;

    // ── uio wiring (SPEC v1.2 §5) ────────────────────────────────────
    //   uio[0] = flash_cs_n  uio[3] = qspi_sck   uio[6] = psram_cs_n
    //   uio[1] = d0          uio[4] = d2         uio[7] = ram_b_disable = 1
    //   uio[2] = d1          uio[5] = d3
    assign uio_out[0] = flash_cs_n_w;
    assign uio_out[1] = qspi_d_out_w[0];
    assign uio_out[2] = qspi_d_out_w[1];
    assign uio_out[3] = qspi_sck_w;
    assign uio_out[4] = qspi_d_out_w[2];
    assign uio_out[5] = qspi_d_out_w[3];
    assign uio_out[6] = psram_cs_n_w;
    assign uio_out[7] = 1'b1;

    assign uio_oe[0] = 1'b1;              // flash_cs_n always out
    assign uio_oe[1] = qspi_d_oe_w[0];    // d0
    assign uio_oe[2] = qspi_d_oe_w[1];    // d1
    assign uio_oe[3] = 1'b1;              // sck always out
    assign uio_oe[4] = qspi_d_oe_w[2];    // d2
    assign uio_oe[5] = qspi_d_oe_w[3];    // d3
    assign uio_oe[6] = 1'b1;              // psram_cs_n always out
    assign uio_oe[7] = 1'b1;              // ram_b_disable = 1 always

    // Inbound QSPI data lines picked off uio_in in the non-contiguous layout.
    assign qspi_d_in_w = {uio_in[5], uio_in[4], uio_in[2], uio_in[1]};

endmodule

`default_nettype wire
