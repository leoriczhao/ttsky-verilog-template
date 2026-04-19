## How it works

**TinyCPU-8** is an 8-bit custom RISC CPU that executes programs from an
external QSPI flash chip. Designed to fit in a single TinyTapeout tile.

- **ISA**: 16-bit fixed-width instructions, 7 general-purpose 8-bit registers
  (R0–R6), hard-wired R7 = 0, 12-bit PC (4096 instruction code space),
  2-bit flags (Z, C). Opcodes: R-type ALU (ADD/SUB/AND/OR/XOR/SHL/SHR/NOT),
  2-operand immediates (ADDI/ANDI/MOVI), conditional branches
  (BZ/BNZ/BC/BNC/BAL), JMP/CALL/RET, IN/OUT, BRK, NOP.
- **Pipeline**: 2-stage (FETCH stalled by QSPI multi-cycle latency, single-cycle
  EXECUTE).
- **External memory**: QSPI flash on uio[5:0] driven in Fast-Read-Quad-IO mode
  (command `0xEB`) at SCK = clk / 2 = 12.5 MHz.
- **Throughput**: ~3–5 MIPS at 25 MHz CPU clock (1 instruction every 5–8 CPU
  cycles, QSPI-bound).

Subroutines are restricted to the lowest 256 instructions of code space
because RET folds the 12-bit PC into the 8-bit R6 link register
(SPEC §3.6 / §8.1).

## How to test

1. **Program a QSPI flash** (W25Q128 on the TinyTapeout QSPI Pmod) with a
   binary produced by `tools/tasm.py`. Instructions are big-endian 16-bit
   words at flash byte addresses `pc << 1`.
2. **Wire the flash** to `uio[5:0]` (CS_n / SCK / D0–D3). Leave `uio[7:6]`
   free — they are hardware-driven status outputs (heartbeat and BRK LED).
3. **Drive `clk` at 25 MHz** and release `rst_n`.
4. The CPU streams instructions from flash and executes them. When BRK is
   encountered, `uio[6]` latches high and `uo_out` is forced to the BRK
   instruction's PC[7:0].
5. Observe `uo_out` for values written by `OUT Rs`. Observe `uio[7]`
   toggling for liveness.

## External hardware

- **TinyTapeout QSPI Pmod** (W25Q128 flash + PSRAMs; PSRAMs unused).
- Optionally LEDs / buttons on `ui_in` / `uo_out` for interactive programs.

## Assembler / example program

A Python assembler (`tools/tasm.py` in the project repo) converts `.s` files
to flash images. Example:

```
        MOVI R0, 0         ; sum
        MOVI R1, 1         ; i
        MOVI R2, 11        ; limit
loop:
        ADD  R0, R0, R1
        ADDI R1, 1
        CMP  R1, R2
        BNZ  loop
        OUT  R0            ; latch 55 on uo_out
        BRK
```
