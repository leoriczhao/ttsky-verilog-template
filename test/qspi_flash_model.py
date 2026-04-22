"""
qspi_flash_model.py — behavioral models for the TinyTapeout QSPI Pmod.

Two chip types on a shared 4-wire bus, distinguished by independent CS lines:

  * `QSPIFlashModel` — W25Q128-class flash (CS = uio[0]).
    Only the `Fast Read Quad I/O` (0xEB) command path is implemented.
    Matches SPEC v1.2 §4.2.

  * `QSPIPsramModel` — APS6404L-class PSRAM (CS = uio[6]).
    Implements Fast Quad Read (0xEB, 6 dummy cycles) and Quad Write (0x38).
    Matches SPEC v1.2 §4.3.

Both share the same bus signals (SCK, D[3:0]). The test harness routes
SCK/D to whichever chip currently has CS low; simultaneous lows are a test
assertion.

The `cocotb` adapter(s) are defined at the bottom for tests that want to
drive a live DUT. Pure-Python unit tests only exercise the core state
machines.
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


# ─── PSRAM model (APS6404L-class) ───────────────────────────────────────────

class QSPIPsramModel:
    """Behavioural APS6404L-class PSRAM, shared-bus companion to the flash.

    Supports two commands:
      * 0xEB Fast Quad Read  — 8 CMD bits (x1), 24-bit ADDR (x4),
                               6 dummy SPI cycles, then streaming quad data.
      * 0x38 Quad Write      — 8 CMD bits (x1), 24-bit ADDR (x4),
                               no dummy, then streaming quad data (writes).

    Each `sck_tick(d_in)` call represents one SCK rising edge. Data output
    timing matches the flash model (see there): during READ the slave starts
    driving the first data nibble on the last dummy tick so the master can
    sample it on the next rising edge.
    """

    CMD_READ  = 0xEB
    CMD_WRITE = 0x38

    CMD_CYCLES   = 8
    ADDR_CYCLES  = 6
    DUMMY_CYCLES = 6          # PSRAM read uses 6 dummies (vs flash's 4)

    def __init__(self, image: bytes = b"", *, size: int = 64 * 1024):
        self.size = size
        self.image = bytearray(image)
        if len(self.image) < size:
            self.image.extend(b"\x00" * (size - len(self.image)))
        self.cs_n = 1
        self.errors: list[str] = []
        self._reset_transaction()

    def load(self, offset: int, data: bytes) -> None:
        end = offset + len(data)
        if end > self.size:
            raise ValueError(f"load past PSRAM size ({end} > {self.size})")
        self.image[offset:end] = data

    def peek(self, offset: int, n: int = 1) -> bytes:
        return bytes(self.image[offset:offset + n])

    def _reset_transaction(self) -> None:
        self.phase = "WAIT_CS"
        self.shift_in = 0
        self.cycles_elapsed = 0
        self.cmd = 0
        self.addr = 0
        self.data_cursor = 0
        self.data_nibble_parity = 0
        self.rx_byte = 0

    def set_cs(self, cs_n: int) -> None:
        cs_n = 1 if cs_n else 0
        if cs_n == 0 and self.cs_n == 1:
            self.phase = "CMD"
            self.shift_in = 0
            self.cycles_elapsed = 0
        elif cs_n == 1 and self.cs_n == 0:
            self._reset_transaction()
        self.cs_n = cs_n

    def sck_tick(self, d_in: int = 0):
        if self.cs_n != 0 or self.phase in ("WAIT_CS", "ERROR"):
            return None

        if self.phase == "CMD":
            self.shift_in = ((self.shift_in << 1) | (d_in & 1)) & 0xFF
            self.cycles_elapsed += 1
            if self.cycles_elapsed == self.CMD_CYCLES:
                self.cmd = self.shift_in
                if self.cmd not in (self.CMD_READ, self.CMD_WRITE):
                    err = f"PSRAM: unsupported command 0x{self.cmd:02X}"
                    self.errors.append(err)
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
                self.data_cursor = self.addr & (self.size - 1)
                self.data_nibble_parity = 0
                self.shift_in = 0
                self.cycles_elapsed = 0
                if self.cmd == self.CMD_READ:
                    self.phase = "DUMMY"
                else:
                    self.phase = "DATA_W"   # write: no dummy
            return None

        if self.phase == "DUMMY":
            self.cycles_elapsed += 1
            if self.cycles_elapsed == self.DUMMY_CYCLES:
                # Last dummy tick emits the first read nibble (like flash).
                self.phase = "DATA_R"
                self.cycles_elapsed = 0
                return self._emit_data_nibble_read()
            return None

        if self.phase == "DATA_R":
            return self._emit_data_nibble_read()

        if self.phase == "DATA_W":
            # Accumulate nibbles; commit on every second tick.
            self.shift_in = ((self.shift_in << 4) | (d_in & 0xF)) & 0xFF
            self.cycles_elapsed += 1
            if self.cycles_elapsed == 2:
                # Full byte received; write and advance.
                if self.data_cursor < self.size:
                    self.image[self.data_cursor] = self.shift_in & 0xFF
                self.data_cursor = (self.data_cursor + 1) & (self.size - 1)
                self.shift_in = 0
                self.cycles_elapsed = 0
            return None

        raise AssertionError(f"PSRAM: bad phase {self.phase!r}")

    def _emit_data_nibble_read(self) -> int:
        if self.data_cursor < self.size:
            byte = self.image[self.data_cursor]
        else:
            byte = 0x00
        if self.data_nibble_parity == 0:
            nib = (byte >> 4) & 0xF
            self.data_nibble_parity = 1
        else:
            nib = byte & 0xF
            self.data_nibble_parity = 0
            self.data_cursor = (self.data_cursor + 1) & (self.size - 1)
        return nib


# ─── combined Pmod model (flash + PSRAM sharing the SCK/D bus) ────────────

class QSPIPmodModel:
    """TinyTapeout QSPI Pmod behavioural model — flash on uio[0] CS, PSRAM
    on uio[6] CS, sharing SCK (uio[3]) and D[3:0] (uio[1,2,4,5]).

    Only one chip may have CS asserted at a time; `sck_tick` routes the
    SPI activity to that chip.
    """

    def __init__(self, flash_image: bytes = b"", psram_image: bytes = b"",
                 psram_size: int = 64 * 1024):
        self.flash = QSPIFlashModel(flash_image)
        self.psram = QSPIPsramModel(psram_image, size=psram_size)

    def set_flash_cs(self, cs_n: int) -> None:
        self.flash.set_cs(cs_n)

    def set_psram_cs(self, cs_n: int) -> None:
        self.psram.set_cs(cs_n)

    def check_cs_exclusive(self) -> None:
        """Assert that at most one chip has CS low. Call after BOTH CS
        lines have been updated in a single edge event."""
        if self.flash.cs_n == 0 and self.psram.cs_n == 0:
            raise AssertionError("CS collision: flash and PSRAM both low")

    def sck_tick(self, d_in: int = 0):
        if self.flash.cs_n == 0:
            return self.flash.sck_tick(d_in)
        if self.psram.cs_n == 0:
            return self.psram.sck_tick(d_in)
        return None


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
