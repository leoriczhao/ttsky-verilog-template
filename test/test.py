# SPDX-FileCopyrightText: © 2026 leoriczhao
# SPDX-License-Identifier: Apache-2.0
#
# Integration test for tt_um_tinycpu8.
#
# Assembles (statically) the classic 1+2+...+10 = 55 program, loads it into a
# simulated QSPI flash model, runs the CPU until BRK fires, and verifies the
# sum was emitted on uo_out.

import os
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Edge, RisingEdge

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from qspi_flash_model import QSPIFlashModel   # noqa: E402


# Pre-assembled program (from tools/tasm.py):
#     MOVI R0, 0       0x8000
#     MOVI R1, 1       0x8201
#     MOVI R2, 11      0x840B
# loop:
#     ADD  R0, R0, R1  0x0008
#     ADDI R1, 1       0x1201
#     CMP  R1, R2      0x0E51
#     BNZ  loop        0x33FC
#     OUT  R0          0x7040
#     BRK              0xE000
ADD_PROGRAM = bytes([
    0x80, 0x00,
    0x82, 0x01,
    0x84, 0x0B,
    0x00, 0x08,
    0x12, 0x01,
    0x0E, 0x51,
    0x33, 0xFC,
    0x70, 0x40,
    0xE0, 0x00,
])


# ─── QSPI flash driver (watches DUT's CS/SCK, feeds back d_in nibbles) ─────

_flash_ref = {'f': None}
_pending   = {'p': None}


def _uio_out(dut):
    try:
        return int(dut.uio_out.value)
    except ValueError:
        return 0xFF


async def _cs_tracker(dut):
    prev = None
    while True:
        await Edge(dut.uio_out)
        cs = _uio_out(dut) & 1
        if cs != prev:
            f = _flash_ref['f']
            if f is not None:
                f.set_cs(cs)
            prev = cs


async def _sck_handler(dut):
    prev = None
    while True:
        await Edge(dut.uio_out)
        v = _uio_out(dut)
        sck = (v >> 1) & 1
        if sck == prev:
            continue
        prev = sck
        f = _flash_ref['f']
        if f is None or f.cs_n != 0:
            continue
        if sck == 1:
            d_out = (v >> 2) & 0xF
            nib = f.sck_tick(d_out)
            if nib is not None:
                _pending['p'] = nib
        else:
            if _pending['p'] is not None:
                try:
                    current = int(dut.uio_in.value)
                except ValueError:
                    current = 0
                # Preserve bits [7:6] and [1:0]; patch [5:2] with the nibble.
                new_val = (current & 0b11000011) | ((_pending['p'] & 0xF) << 2)
                dut.uio_in.value = new_val
                _pending['p'] = None


@cocotb.test()
async def test_add_loop(dut):
    """1+2+...+10 should land on uo_out as 55 before BRK halts the CPU."""
    dut._log.info("Start TinyCPU-8 integration test")

    # 25 MHz CPU clock per SPEC §6 (40 ns period).
    cocotb.start_soon(Clock(dut.clk, 40, unit="ns").start())

    # Load program into the flash model.
    flash = QSPIFlashModel()
    flash.load(0, ADD_PROGRAM)
    _flash_ref['f'] = flash
    _pending['p']   = None

    cocotb.start_soon(_cs_tracker(dut))
    cocotb.start_soon(_sck_handler(dut))

    # Reset.
    dut.ena.value   = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # Spin until BRK fires (uio_out[6] high). Capture the last non-zero uo_out,
    # which is the `OUT R0` result right before BRK overrides uo_out with PC[7:0].
    last_output = 0
    for _ in range(30000):
        await RisingEdge(dut.clk)
        if (_uio_out(dut) >> 6) & 1:
            break
        try:
            v = int(dut.uo_out.value)
        except ValueError:
            v = 0
        if v != 0:
            last_output = v
    else:
        raise TimeoutError("CPU never reached BRK within 30k cycles")

    dut._log.info(f"Observed OUT value: {last_output}")
    assert last_output == 55, f"Expected sum 1..10 = 55, got {last_output}"
