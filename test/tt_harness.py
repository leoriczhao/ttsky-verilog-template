"""
tt_harness.py — shared cocotb driver for tt_um_tinycpu8 v1.2.

Drives the TinyTapeout QSPI Pmod pinout (SPEC v1.2 §5):

    uio[0] = flash_cs_n      uio[3] = qspi_sck       uio[6] = psram_cs_n
    uio[1] = d0 / MOSI       uio[4] = d2             uio[7] = ram_b_disable
    uio[2] = d1 / MISO       uio[5] = d3

Responsibilities:
  * Watches uio_out for CS line changes; routes to flash or PSRAM model.
  * On SCK rising edges, ticks the active model with d_out from the DUT.
  * On SCK falling edges, drives the nibble the model emitted back onto
    uio_in[1,2,4,5].
"""

import os
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Edge, RisingEdge, Timer

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from qspi_flash_model import QSPIPmodModel   # noqa: E402


_pmod_ref = {'p': None}
_pending  = {'p': None}


def _uio_out(dut):
    try:
        return int(dut.uio_out.value)
    except ValueError:
        return 0xC1     # both CS high idle (flash=1, psram=1, rest don't care)


def _unpack_dlines(v):
    """Read d[3:0] as a 4-bit nibble out of uio bit-positions 1,2,4,5."""
    d0 = (v >> 1) & 1
    d1 = (v >> 2) & 1
    d2 = (v >> 4) & 1
    d3 = (v >> 5) & 1
    return (d3 << 3) | (d2 << 2) | (d1 << 1) | d0


def _pack_dlines(current_uio_in, nibble):
    """Patch bits 1,2,4,5 of uio_in with the nibble; preserve 0,3,6,7."""
    d0 = nibble & 1
    d1 = (nibble >> 1) & 1
    d2 = (nibble >> 2) & 1
    d3 = (nibble >> 3) & 1
    cleared = current_uio_in & ~0b00110110
    return (cleared
            | (d0 << 1) | (d1 << 2)
            | (d2 << 4) | (d3 << 5))


async def _cs_tracker(dut):
    prev_flash = None
    prev_psram = None
    while True:
        await Edge(dut.uio_out)
        # Let downstream combinational paths settle in case iverilog has
        # not fully propagated all delta-cycle re-evaluations yet.
        await Timer(1, unit="ps")
        v = _uio_out(dut)
        flash_cs = v & 1
        psram_cs = (v >> 6) & 1
        pmod = _pmod_ref['p']
        if pmod is None:
            continue
        # Apply CS changes "rises first, then falls" — prevents a spurious
        # both-low intermediate when the DUT swaps between flash and PSRAM in
        # a single combinational edge.
        if flash_cs == 1 and prev_flash == 0:
            pmod.set_flash_cs(1)
        if psram_cs == 1 and prev_psram == 0:
            pmod.set_psram_cs(1)
        if flash_cs == 0 and prev_flash != 0:
            pmod.set_flash_cs(0)
        if psram_cs == 0 and prev_psram != 0:
            pmod.set_psram_cs(0)
        # Only now — after ALL updates — check the genuine invariant.
        try:
            pmod.check_cs_exclusive()
        except AssertionError as e:
            import cocotb as _c
            fsm_state = int(dut.u_fetch.state.value) if hasattr(dut, 'u_fetch') else -1
            _c.log.warning(
                f"CS-exclusive violated at flash={flash_cs} psram={psram_cs} "
                f"v={v:#x} prev={prev_flash},{prev_psram} state={fsm_state:#x}")
            raise
        prev_flash = flash_cs
        prev_psram = psram_cs


async def _sck_handler(dut):
    prev_sck = None
    tick = 0
    while True:
        await Edge(dut.uio_out)
        v = _uio_out(dut)
        sck = (v >> 3) & 1
        if sck == prev_sck:
            continue
        prev_sck = sck
        pmod = _pmod_ref['p']
        if pmod is None:
            continue
        if pmod.flash.cs_n != 0 and pmod.psram.cs_n != 0:
            continue
        if sck == 1:
            tick += 1
            d_out = _unpack_dlines(v)
            active = "flash" if pmod.flash.cs_n == 0 else "psram"
            phase = (pmod.flash.phase if active == "flash" else pmod.psram.phase)
            nib = pmod.sck_tick(d_out)
            dut._log.debug(
                f"t{tick}↑ {active} phase={phase} d_out={d_out:#x} emit={nib}")
            if nib is not None:
                _pending['p'] = nib
        else:
            if _pending['p'] is not None:
                try:
                    current = int(dut.uio_in.value)
                except ValueError:
                    current = 0
                dut.uio_in.value = _pack_dlines(current, _pending['p'])
                _pending['p'] = None


async def init_and_reset(dut, pmod, *, ui_in_value=0, clock_period_ns=40):
    """Initialise the DUT, load the PmodModel, start clock and drivers."""
    dut.ui_in.value  = ui_in_value
    dut.uio_in.value = 0
    dut.ena.value    = 1
    dut.rst_n.value  = 0
    _pmod_ref['p']   = pmod
    _pending['p']    = None
    cocotb.start_soon(Clock(dut.clk, clock_period_ns, unit="ns").start())
    cocotb.start_soon(_cs_tracker(dut))
    cocotb.start_soon(_sck_handler(dut))
    for _ in range(10):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


def brk_halted(dut):
    """BRK signature: BOTH chip selects high (flash & PSRAM idle).
    We can't use flash_cs alone because it also goes high during a
    PSRAM LOAD/STORE transaction."""
    v = _uio_out(dut)
    flash_cs = v & 1
    psram_cs = (v >> 6) & 1
    return flash_cs == 1 and psram_cs == 1


async def run_to_brk(dut, timeout_cycles, per_cycle=None):
    """Run until the flash CS goes high and stays high for a while
    (heuristic for BRK halt). Optionally call per_cycle(dut, cycle)."""
    for cy in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if per_cycle is not None:
            per_cycle(dut, cy)
        # After a reasonable startup (to let the first fetch run), check for
        # the CS-high-plus-uo_out-stable signature.
    # We return after timeout_cycles total regardless; caller should
    # inspect dut.uo_out and flash CS for BRK.


async def wait_for_halt(dut, timeout_cycles, per_cycle=None):
    """Wait until flash CS goes high AND stays high for several cycles.
    BRK in v1.2 halts the fetch FSM, making flash_cs_n pull high and stay."""
    idle_run = 0
    for cy in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if per_cycle is not None:
            per_cycle(dut, cy)
        if brk_halted(dut):
            idle_run += 1
            if idle_run >= 30:
                return cy
        else:
            idle_run = 0
    raise TimeoutError(f"BRK halt not detected within {timeout_cycles} cycles")
