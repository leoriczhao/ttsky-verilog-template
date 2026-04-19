"""
qspi_flash_model.py — behavioral model of a W25Q128-like QSPI flash supporting
only the `Fast Read Quad I/O` (0xEB) command path used by TinyCPU-8 (SPEC v1.1
§4).

Design:

  * `QSPIFlashModel` — pure-Python state machine, per-SCK tick. Unit-testable
    without cocotb.
  * `drive_qspi_flash(...)` — cocotb coroutine adapter that samples the DUT's
    CS / SCK / D3..D0 pins and calls into the model.

Timing model (simplified SPI mode 0, behavioral):

  * CS falling edge → begin a transaction.
      - If `continuous_read` is set from a prior mode byte 0xAx, skip CMD.
      - Otherwise start in CMD phase (x1, D0 input).
  * Each call to `sck_tick(d_in)` represents one SCK rising edge:
      - Inbound phases (CMD/ADDR/MODE): sample `d_in`; return None.
      - DUMMY: 4 ticks, return None.
      - DATA: each tick returns one nibble (high nibble of a byte first),
              until CS rises.
  * CS rising edge → end transaction and return to WAIT_CS.

Addressing: the flash is byte-addressed. Instruction byte 0 of word N lives at
byte address `N*2` (high byte); byte 1 at `N*2+1` (low byte). See SPEC §4.4, §4.6.
"""

from __future__ import annotations

import logging
from typing import Optional

log = logging.getLogger("qspi_flash")


# ─── pure-Python core ──────────────────────────────────────────────────────

class QSPIFlashModel:
    """Behavioral W25Q128-family flash supporting 0xEB Fast Read Quad I/O.

    Public mutating methods:
        set_cs(cs_n)           — driver lifts/lowers CS
        sck_tick(d_in) -> int  — driver issues one SCK rising edge; returns
                                  the nibble the device drives back
                                  (None = tristate / not in data phase)

    Public helpers:
        load(offset, data)     — patch bytes into the flash image
        peek(offset, n)        — read bytes without side effects
        word_at(instr_addr)    — 16-bit word at instruction address
    """

    CMD_FAST_READ_QUAD_IO = 0xEB

    # Phase-length constants (in SCK cycles)
    CMD_CYCLES   = 8          # 8 bits, x1
    ADDR_CYCLES  = 6          # 24 bits, x4
    MODE_CYCLES  = 2          # 8 bits,  x4
    DUMMY_CYCLES = 4
    # Data phase is open-ended — streams until CS rises

    def __init__(self, image: bytes = b"", *, size: int = 16 * 1024 * 1024):
        self.size = size
        self.image = bytearray(image)
        # Pad erased cells with 0xFF up to size on demand (lazy)
        self.cs_n: int = 1
        self.continuous_read: bool = False
        self.errors: list[str] = []
        self._reset_transaction()

    # ── image manipulation ────────────────────────────────────────────────

    def load(self, offset: int, data: bytes) -> None:
        """Patch `data` into the image starting at byte `offset`."""
        end = offset + len(data)
        if end > self.size:
            raise ValueError(f"load past flash size ({end} > {self.size})")
        if end > len(self.image):
            self.image.extend(b"\xFF" * (end - len(self.image)))
        self.image[offset:end] = data

    def peek(self, offset: int, n: int = 1) -> bytes:
        """Read `n` bytes starting at `offset`, treating uninitialized as 0xFF."""
        out = bytearray()
        for i in range(n):
            a = offset + i
            out.append(self.image[a] if a < len(self.image) else 0xFF)
        return bytes(out)

    def word_at(self, instr_addr: int) -> int:
        """16-bit instruction word at instruction (word) address `instr_addr`.

        Matches SPEC §4.6 endianness: byte[2N]=high, byte[2N+1]=low.
        """
        hi, lo = self.peek(instr_addr * 2, 2)
        return (hi << 8) | lo

    # ── state helpers ─────────────────────────────────────────────────────

    def _reset_transaction(self) -> None:
        self.phase: str = "WAIT_CS"
        self.shift_in: int = 0
        self.cycles_elapsed: int = 0
        self.addr: int = 0
        self.mode_byte: int = 0
        self.data_cursor: int = 0   # byte offset into image for next data nibble pair
        self.data_nibble_parity: int = 0   # 0 = next output is high nibble

    def _begin_transaction(self) -> None:
        self.phase = "ADDR" if self.continuous_read else "CMD"
        self.shift_in = 0
        self.cycles_elapsed = 0
        self.addr = 0
        self.mode_byte = 0
        self.data_cursor = 0
        self.data_nibble_parity = 0

    # ── driver interface ──────────────────────────────────────────────────

    def set_cs(self, cs_n: int) -> None:
        """Drive CS#. 0 = selected. Call on each CS level change."""
        cs_n = 1 if cs_n else 0
        if cs_n == 0 and self.cs_n == 1:
            self._begin_transaction()
        elif cs_n == 1 and self.cs_n == 0:
            self._reset_transaction()
        self.cs_n = cs_n

    def sck_tick(self, d_in: int = 0) -> Optional[int]:
        """One SCK rising edge. `d_in` is the 4-bit master-driven value of D3..D0.

        Returns:
            nibble (0..15) the device drives on D3..D0 this cycle, or
            None if device is tristated / phase ignores output.
        """
        if self.cs_n != 0 or self.phase in ("WAIT_CS", "ERROR"):
            return None

        if self.phase == "CMD":
            d0 = d_in & 1
            self.shift_in = ((self.shift_in << 1) | d0) & 0xFF
            self.cycles_elapsed += 1
            if self.cycles_elapsed == self.CMD_CYCLES:
                if self.shift_in != self.CMD_FAST_READ_QUAD_IO:
                    err = f"unsupported command 0x{self.shift_in:02X}"
                    self.errors.append(err)
                    log.warning(err)
                    self.phase = "ERROR"
                    return None
                self.phase = "ADDR"
                self.shift_in = 0
                self.cycles_elapsed = 0
            return None

        if self.phase == "ADDR":
            self.shift_in = ((self.shift_in << 4) | (d_in & 0xF)) & 0xFFFFFF
            self.cycles_elapsed += 1
            if self.cycles_elapsed == self.ADDR_CYCLES:
                self.addr = self.shift_in & 0xFFFFFF
                self.data_cursor = self.addr
                self.phase = "MODE"
                self.shift_in = 0
                self.cycles_elapsed = 0
            return None

        if self.phase == "MODE":
            self.shift_in = ((self.shift_in << 4) | (d_in & 0xF)) & 0xFF
            self.cycles_elapsed += 1
            if self.cycles_elapsed == self.MODE_CYCLES:
                self.mode_byte = self.shift_in & 0xFF
                self.continuous_read = ((self.mode_byte >> 4) & 0xF) == 0xA
                self.phase = "DUMMY"
                self.cycles_elapsed = 0
            return None

        if self.phase == "DUMMY":
            self.cycles_elapsed += 1
            if self.cycles_elapsed == self.DUMMY_CYCLES:
                # Real hardware turns the bus around during the LAST dummy
                # cycle — the slave starts driving data on the tail end so the
                # master can sample the first nibble on the NEXT SCK rising
                # edge. Emit the first data nibble on this tick.
                self.phase = "DATA"
                self.data_nibble_parity = 0
                return self._emit_data_nibble()
            return None

        if self.phase == "DATA":
            return self._emit_data_nibble()

        raise AssertionError(f"unexpected phase {self.phase!r}")

    def _emit_data_nibble(self) -> int:
        """Return the next data nibble, advancing the byte cursor."""
        if self.data_cursor < len(self.image):
            byte = self.image[self.data_cursor]
        else:
            byte = 0xFF   # erased
        if self.data_nibble_parity == 0:
            nib = (byte >> 4) & 0xF
            self.data_nibble_parity = 1
        else:
            nib = byte & 0xF
            self.data_nibble_parity = 0
            self.data_cursor = (self.data_cursor + 1) & (self.size - 1)
        return nib


# ─── image helpers ──────────────────────────────────────────────────────────

def image_from_words(words) -> bytes:
    """Encode a sequence of 16-bit instruction words to big-endian bytes."""
    out = bytearray()
    for w in words:
        if not 0 <= w <= 0xFFFF:
            raise ValueError(f"word {w:#x} out of 16-bit range")
        out.append((w >> 8) & 0xFF)
        out.append(w & 0xFF)
    return bytes(out)


def words_from_image(image: bytes) -> list:
    """Decode byte stream into list of 16-bit big-endian words."""
    if len(image) % 2 != 0:
        raise ValueError("image length must be even")
    return [(image[2 * i] << 8) | image[2 * i + 1] for i in range(len(image) // 2)]


# ─── cocotb driver (optional) ───────────────────────────────────────────────

try:
    import cocotb
    from cocotb.triggers import Edge, RisingEdge, FallingEdge
    _HAS_COCOTB = True
except ImportError:       # pragma: no cover — cocotb not installed in pure-unit env
    _HAS_COCOTB = False


if _HAS_COCOTB:

    async def drive_qspi_flash(flash: QSPIFlashModel,
                               cs_signal,
                               sck_signal,
                               d_out_signal,
                               d_in_signal) -> None:
        """Cocotb coroutine that wires a `QSPIFlashModel` to DUT pins.

        Args:
            flash:        model instance (already loaded with program image).
            cs_signal:    bit that the DUT drives as CS# (e.g. dut.uio_out[0]).
            sck_signal:   bit that the DUT drives as SCK (e.g. dut.uio_out[1]).
            d_out_signal: 4-bit slice the DUT drives for D3..D0 outputs
                          (e.g. dut.uio_out.value[5:2]).
            d_in_signal:  4-bit slice the driver (this coroutine) drives back
                          to the DUT for data-phase responses
                          (e.g. dut.uio_in[5:2]).

        Timing: the driver samples data on the SCK rising edge and updates the
        response on the SCK falling edge — matching SPI mode 0.
        """
        # Seed with current CS level.
        flash.set_cs(int(cs_signal.value))

        async def _track_cs():
            while True:
                await Edge(cs_signal)
                flash.set_cs(int(cs_signal.value))
                # Tristate input on CS high (writing X; matches "floating" semantic)
                if flash.cs_n == 1:
                    try:
                        d_in_signal.value = 0
                    except Exception:
                        pass

        cocotb.start_soon(_track_cs())

        while True:
            await RisingEdge(sck_signal)
            if flash.cs_n != 0:
                continue
            try:
                d_in = int(d_out_signal.value)
            except ValueError:
                d_in = 0   # 'x' or 'z' on master side; treat as 0 for inbound shift
            nibble = flash.sck_tick(d_in)
            if nibble is not None:
                # Drive the response on the *next* falling edge for proper
                # SPI mode-0 setup-before-sample behavior.
                await FallingEdge(sck_signal)
                d_in_signal.value = nibble
