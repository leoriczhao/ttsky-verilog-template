## How it works

**Jimu-8** is an 8-bit custom RISC CPU (v1.2) that fetches instructions
from an external QSPI flash and uses a QSPI PSRAM as 64 KB of byte-
addressable data memory. Designed to fit in a single TinyTapeout tile.

- **ISA**: 16-bit fixed-width instructions, 7 general-purpose 8-bit
  registers (R0–R6), hard-wired R7 = 0, 12-bit PC (4096 instruction code
  space = 8 KB of code), 2-bit flags (Z, C).
- **Opcodes**: R-type ALU (ADD / SUB / AND / OR / XOR / SHL / SHR / NOT),
  2-operand immediates (ADDI / ANDI / MOVI), conditional branches
  (BZ / BNZ / BC / BNC / BAL), JMP / CALL / RET, **LOAD / STORE** to
  64 KB PSRAM, IN / OUT, BRK, NOP.
- **CALL / RET** use a 2-register link pair: `R5[3:0]` holds PC+1's
  high nibble, `R6[7:0]` holds the low byte. Full 12-bit return PC is
  reconstructed on RET — subroutines can live anywhere in the 4 KW
  code space.
- **Pipeline**: 2-stage; FETCH stalled by QSPI multi-cycle latency,
  single-cycle EXECUTE. LOAD / STORE stall the pipe for ~30 CPU cycles
  while the PSRAM transaction runs.
- **External memory**: 4-wire QSPI, shared by flash (CS on `uio[0]`,
  Fast Read Quad I/O 0xEB) and PSRAM-A (CS on `uio[6]`, Fast Quad Read
  0xEB + Quad Write 0x38). SCK = CPU clock / 2 = 12.5 MHz.
- **Throughput**: ~3–5 MIPS at 25 MHz CPU clock for pure ALU/branch code;
  ~30 cycles per LOAD or STORE.

## How to test

1. **Program a QSPI flash** (W25Q128-class on the TinyTapeout QSPI Pmod)
   with a binary from `tools/tasm.py`. Instructions are big-endian 16-bit
   words at flash byte address `pc << 1`.
2. **Connect the QSPI Pmod**: flash CS on `uio[0]`, SCK on `uio[3]`, data
   on `uio[1,2,4,5]`, PSRAM-A CS on `uio[6]`, `uio[7]` = 1 (PSRAM-B is
   disabled). Pinout matches the Pmod silkscreen.
3. **Drive `clk` at 25 MHz** and release `rst_n`.
4. The CPU streams instructions from flash and executes them. LOAD /
   STORE transactions route the shared bus to PSRAM-A automatically
   (both chips never have CS low simultaneously).
5. **BRK** halts the CPU, raises both CS lines high, and latches
   `uo_out` = `PC[7:0]` of the BRK instruction (persistent until reset).
6. Observe `uo_out` for values written by `OUT Rs`. External observers
   detect halt by seeing both CS inactive + stable `uo_out`.

## External hardware

- **TinyTapeout QSPI Pmod** — flash (W25Q128) + PSRAM-A (APS6404L-class)
  sharing SCK / D[3:0]. PSRAM-B on the same Pmod is disabled by this
  design (`uio[7]` hardwired high).
- Optional LEDs / buttons on `ui_in` / `uo_out` for interactive programs
  (Fibonacci-over-UART, bit-bang keyboard, etc.).

## Assembler / example program

A Python assembler (`tools/tasm.py` in the project repo) converts `.s`
files to flash images. Example — STORE / LOAD round-trip via PSRAM:

```
        MOVI  R0, 0xAB       ; data byte
        MOVI  R1, 0          ; Rhi (address high)
        MOVI  R2, 0x20       ; Rlo (address low)

        STORE R0, R1, R2     ; PSRAM[0x0020] <- 0xAB

        MOVI  R0, 0          ; clear so LOAD is observable
        LOAD  R0, R1, R2     ; R0 <- PSRAM[0x0020]  (expect 0xAB)

        OUT   R0             ; emit on uo_out
        BRK
```

Design repository and full spec (ISA / ABI / microarchitecture /
decisions): `docs/ISA.md`, `docs/ABI.md`, `docs/MICROARCH.md`,
`docs/PINOUT.md`, `docs/DECISIONS.md`.
