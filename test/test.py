# SPDX-FileCopyrightText: © 2026 leoriczhao
# SPDX-License-Identifier: Apache-2.0
#
# Jimu-8 v1.2 integration tests for TinyTapeout CI.
#
# Runs three pre-assembled programs against the dual-chip QSPI Pmod model
# (flash + PSRAM-A), exercising:
#
#   1. test_add_loop         — 1+2+…+10 = 55 via ADD/ADDI/CMP/BNZ.
#   2. test_ram_basic        — STORE 0xAB to PSRAM[0x0020]; LOAD back; OUT.
#   3. test_call_high_addr   — CALL from PC 0x100 → subroutine at 0x001;
#                              verifies the v1.2 12-bit CALL/RET link
#                              (R5 high nibble + R6 low byte).
#
# Programs are inlined as bytes so this file has no dependency on the
# assembler (`tools/tasm.py`) at CI time.

import os
import sys

import cocotb

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from qspi_flash_model import QSPIPmodModel   # noqa: E402
from tt_harness import init_and_reset, wait_for_halt   # noqa: E402


# ───────────────────────── programs (pre-assembled) ────────────────────────

# add_program.s — sum 1..10 = 55, OUT R0, BRK.
ADD_PROGRAM = bytes([
    0x80, 0x00,   # MOVI R0, 0        ; sum = 0
    0x82, 0x01,   # MOVI R1, 1        ; i = 1
    0x84, 0x0B,   # MOVI R2, 11       ; limit
    0x00, 0x08,   # ADD  R0, R0, R1
    0x12, 0x01,   # ADDI R1, 1
    0x0E, 0x51,   # CMP  R1, R2       ; SUB R7, R1, R2  → sets Z when i==11
    0x33, 0xFC,   # BNZ  loop (-4)
    0x70, 0x40,   # OUT  R0
    0xE0, 0x00,   # BRK
])

# ram_basic.s — STORE 0xAB to PSRAM[0x0020], LOAD back, OUT, BRK.
RAM_BASIC = bytes([
    0x80, 0xAB,   # MOVI R0, 0xAB     ; data
    0x82, 0x00,   # MOVI R1, 0        ; Rhi
    0x84, 0x20,   # MOVI R2, 0x20     ; Rlo
    0xA0, 0x50,   # STORE R0, R1, R2  ; mem[0x0020] <- 0xAB
    0x80, 0x00,   # MOVI R0, 0        ; clear so the LOAD is observable
    0x90, 0x50,   # LOAD  R0, R1, R2  ; R0 <- mem[0x0020]
    0x70, 0x40,   # OUT  R0           ; expect 0xAB on uo_out
    0xE0, 0x00,   # BRK
])

# call_high_addr.s — JMP main ; inc: ADDI R0,1 ; RET ; ... pad ... ;
#                    main (at PC 0x100): MOVI R0,0; CALL inc×2; OUT R0; BRK.
_chdr_head = bytes([
    0x41, 0x00,   # JMP  main (target12 = 0x100)
    0x10, 0x01,   # inc: ADDI R0, 1
    0x60, 0x00,   # RET
])
_chdr_main = bytes([
    0x80, 0x00,   # MOVI R0, 0
    0x50, 0x01,   # CALL inc  (target12 = 0x001)
    0x50, 0x01,   # CALL inc
    0x70, 0x40,   # OUT  R0           ; expect 2
    0xE0, 0x00,   # BRK
])
# main at instruction addr 0x100 = byte 0x200. Pad with 0xFF (NOP).
CALL_HIGH_ADDR = (_chdr_head
                  + b"\xFF" * (0x200 - len(_chdr_head))
                  + _chdr_main)


# ─────────────────────────────── tests ─────────────────────────────────────

@cocotb.test()
async def test_add_loop(dut):
    """1+2+…+10 = 55 emitted on uo_out before BRK."""
    pmod = QSPIPmodModel(flash_image=ADD_PROGRAM)
    await init_and_reset(dut, pmod)

    saw_target = False
    def track(_d, _c):
        nonlocal saw_target
        try:
            v = int(_d.uo_out.value)
        except ValueError:
            return
        if v == 55:
            saw_target = True

    await wait_for_halt(dut, 10000, per_cycle=track)
    assert saw_target, "add_loop: uo_out never latched 55"


@cocotb.test()
async def test_ram_basic(dut):
    """STORE 0xAB to PSRAM[0x0020]; LOAD back; expect 0xAB on uo_out."""
    pmod = QSPIPmodModel(flash_image=RAM_BASIC)
    await init_and_reset(dut, pmod)

    saw_target = False
    def track(_d, _c):
        nonlocal saw_target
        try:
            v = int(_d.uo_out.value)
        except ValueError:
            return
        if v == 0xAB:
            saw_target = True

    await wait_for_halt(dut, 5000, per_cycle=track)

    psram_val = pmod.psram.peek(0x20, 1)
    assert psram_val == b"\xAB", (
        f"STORE failed: PSRAM[0x0020]={psram_val.hex()} (expected ab)")
    assert saw_target, "ram_basic: uo_out never latched 0xAB"


@cocotb.test()
async def test_call_high_addr(dut):
    """CALL from PC 0x100 to subroutine at 0x001 (twice); expect R0 = 2.

    Verifies the v1.2 12-bit CALL/RET link pair — a v1.1 8-bit-only link
    would drop the 0x1 high nibble and loop forever.
    """
    pmod = QSPIPmodModel(flash_image=CALL_HIGH_ADDR)
    await init_and_reset(dut, pmod)

    saw_target = False
    def track(_d, _c):
        nonlocal saw_target
        try:
            v = int(_d.uo_out.value)
        except ValueError:
            return
        if v == 2:
            saw_target = True

    await wait_for_halt(dut, 20000, per_cycle=track)
    assert saw_target, "call_high_addr: uo_out never latched 2 (RET mis-lands)"
