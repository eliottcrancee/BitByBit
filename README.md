# PyTransCPU

**From a single transistor to a working SAP-1 CPU — simulated in Python.**

PyTransCPU is an educational, bottom-up simulation of a complete CPU. It
models a machine literally from its most elementary physical components: PMOS
and NMOS transistors. On top of them it builds every logic gate, every
multiplexer, every adder, every flip-flop, and finally a full computer — the
SAP-1 ("Simple As Possible 1") described in classic textbooks and popularized
by the Ben Eater breadboard-computer series.

The project is deliberately *naive*: it never relies on Python's built-in
boolean operators inside the hardware layer. `not`, `and`, `or` and high-level
tricks are forbidden when building the logic gates; everything is wired by
hand from transistors, exactly the way real digital hardware is built.

---

## The signal model

Hardware only deals with two electrical states, plus one special state for
"disconnected" wires:

| Value | Meaning                                  | Python   |
|-------|------------------------------------------|----------|
| `1`   | HIGH voltage (`VCC`, supply rail)        | `HIGH`   |
| `0`   | LOW voltage (`GND`, ground)              | `LOW`    |
| `None`| Z / high-impedance (floating wire)       | `Z`      |

Bits are always stored *least-significant bit first*. For instance the byte
`(1, 0, 1)` represents the value `5`.

This model is deliberately numeric and pedagogical. It does not simulate
real voltages, currents, capacitances or propagation latencies.

---

## Architecture: a bottom-up stack

Everything is assembled tier by tier. Each layer only uses the one below it:

```
transistors (NMOS / PMOS)
        |
        v
logic gates (NOT, NAND, NOR, AND, OR, XOR, XNOR)
        |
        v
multiplexers / decoders
        |
        v
adders and the ALU
        |
        v
latches, flip-flops, registers, RAM
        |
        v
CPU (SAP-1)
```

### Project layout

```
python/                # Python implementation (uv-managed)
    pytranscpu/
        hardware.py    # transistors, wire, bus, signal model, cost, helpers
    gates.py       # logic gates built from transistors
    mux.py         # 2:1 and 8-bit multiplexers
    decoder.py     # 2-to-4 and 4-to-16 decoders
    arithmetic.py  # half/full adders, 8-bit adder, ALU
    latches.py     # SR latch, D latch, D flip-flops, ring counter
        memory.py      # 8-bit register, program counter, 256-bit RAM
        cpu_sap1.py    # control unit + the whole SAP-1 computer

    test/              # pytest suite for every module

rust/                  # Rust implementation (Cargo), fully independent
```

---

## Modules

### `hardware.py` — the physical primitives

The lowest layer knows neither instructions nor assemblers. It provides:

- the **NMOS** and **PMOS** transistor primitives, modelled as callables that
gate a `source` towards their `drain` based on a `gate` signal;
- **`wire`**, which connects several outputs (with high-impedance handling)
and `bus8`, which resolves an 8-bit shared bus;
- the `Bit` / `Byte` / `Bus` types and the `HIGH`, `LOW`, `VCC`, `GND`,
`HIGH_IMPEDANCE` constants;
- **`HardwareCost`** — a simple transistor / memory-bit counter, exposed by
every component;
- helpers such as `bits_to_int`, `int_to_bits` and `stabilize` (for feedback
circuits like latches);
- a set of explicit exceptions for hardware errors (invalid signals, bus
conflicts, unstable circuits).

### `gates.py` — logic gates

The three primitive gates are wired at the transistor level:
`NotGate` (1 PMOS + 1 NMOS), `NandGate`, `NorGate`. Every other gate is
composed from them: `AndGate`, `OrGate`, `XorGate`, `XnorGate`.

### `mux.py` — multiplexers

`Mux2x1` selects one of two inputs with a `select` line
`(a AND NOT select) OR (b AND select)`; `Mux8bits2x1` places eight of them in
parallel.

### `decoder.py` — decoders

`Decoder2to4` activates exactly one output line for a 2-bit input;
`Decoder4to16` cascades two of them. These decode RAM addresses.

### `arithmetic.py` — adders and the ALU

`HalfAdder`, `FullAdder` and `Adder8Bits` (an 8-bit ripple-carry chain) build
up to `ALU8Bits`. The ALU computes `A + B` or `A - B` depending on a
`subtract` signal (two's-complement via `A + NOT B + 1`) and exposes a **carry**
flag and a **zero** flag. Its result bus is gated by a `load` signal so it can
float in high impedance and share the bus with other drivers.

### `latches.py` — sequential memory

The sequential (stateful) components:

| Component              | Purpose                                  |
|------------------------|------------------------------------------|
| `SRLatch`              | two cross-coupled NOR gates, one bit     |
| `DLatch`               | transparent while `enable` is HIGH       |
| `DFlipFlop`            | updates only on a rising clock edge      |
| `DFlipFlopSave`        | write gated by a `save` signal           |
| `DFlipFlopSaveLoad`    | save, with `load`-gated outputs (tri-state)|
| `OneHotCounter6Bits`   | 6-bit ring counter (the micro-step sequencer) |

### `memory.py` — storage components

- `Register8Bits` — eight D flip-flops with save/load;
- `ProgramCounter4Bits` — self-incrementing 4-bit counter, with load;
- `Ram256Bits` — 16 registers of 8 bits, addressed by a 4-to-16 decoder
(256 physical bits).

### `cpu_sap1.py` — the SAP-1 computer

The final assembly: a `ProgramCounter`, `MemoryAddressRegister`, `Instruction
Register`, `Accumulator`, `B Register`, `ALU`, `RAM`, a 6-state micro-step
sequencer, and a `ControlUnit` that generates every control signal purely from
logic gates. It performs the classic **fetch-decode-execute** cycle.

---

## The SAP-1 instruction set

Instructions are single bytes: the high nibble is the opcode, the low nibble is
the memory operand.

| Opcode (bin) | Hex | Instruction | Effect                     |
|--------------|-----|-------------|----------------------------|
| `0000`       | `0` | `NOP`       | no operation               |
| `0001`       | `1` | `LDA`       | `A <- MEM[operand]`        |
| `0010`       | `2` | `ADD`       | `A <- A + MEM[operand]`    |
| `0011`       | `3` | `SUB`       | `A <- A - MEM[operand]`    |
| `0100`       | `4` | `STA`       | `MEM[operand] <- A`        |
| `0101`       | `5` | `LDI`       | `A <- constant (operand)`  |
| `0110`       | `6` | `JMP`       | `PC <- operand`            |
| `0111`       | `7` | `JZ`        | `PC <- operand if ACC == 0`|
| `1000`       | `8` | `JC`        | `PC <- operand if CARRY`   |
| `1111`       | `F` | `HLT`       | halt the computer          |

---

## Installation

The project is managed with [uv](https://github.com/astral-sh/uv) and targets
Python 3.13+.

```powershell
# Create the environment and install dependencies
uv sync

# Activate it (or prefix any command with `uv run`)
.venv\Scripts\Activate.ps1
```

---

## Quick start

Build the CPU, load a small program into RAM and run it:

```python
from pytranscpu.cpu_sap1 import SAP1
from pytranscpu.hardware import bits_to_int

cpu = SAP1()
print("Transistors:", cpu.transistor_count)
print("Memory bits:", cpu.bit_count)

# Load a program: LDI 10, HLT
cpu.load_program([0x5A, 0xF0])
cpu.run()

print("Halted:", cpu.halted)
print("Accumulator:", bits_to_int(cpu.out))  # -> 10
```

A commented multiply-by-loop example lives in `python/debug_loop.py`.

### Driving the clock manually

The shared clock and the micro-step sequencer can be advanced by hand, which is
useful for understanding the internal state transitions:

```python
cpu = SAP1()
cpu.load_program([0x57, 0xF0])   # LDI 7, HLT
cpu.clock_tick()      # toggle the shared clock, returns the new value
cpu.update()          # perform one micro-state transition
cpu.micro_step()      # advance the sequencer by one micro-state
cpu.step_instruction()  # advance until the next fetch (or halted)
cpu.run()             # run until halted (or max_steps)
```

Register and RAM contents are inspectable at any time, e.g.
`cpu.accumulator.state`, `cpu.program_counter.state[:4]`,
`cpu.zero_flag.state[0]`, `cpu.ram.registers[0xD].state`.

---

## Examples

### A loop that multiplies 3 × 5, then halts

```python
from pytranscpu.cpu_sap1 import SAP1
from pytranscpu.hardware import bits_to_int

cpu = SAP1()
program = [
    0x1F,  # 0: LDA 0xF   (load running sum)
    0x2E,  # 1: ADD 0xE   (add 3)
    0x4F,  # 2: STA 0xF   (store back to sum)
    0x1D,  # 3: LDA 0xD   (load count)
    0x3C,  # 4: SUB 0xC   (subtract 1)
    0x79,  # 5: JZ 9      (if count reached 0, jump)
    0x4D,  # 6: STA 0xD   (save decremented count)
    0x60,  # 7: JMP 0     (loop)
    0x00,  # 8: NOP
    0x1F,  # 9: LDA 0xF   (load final sum)
    0xF0,  # A: HLT
    0x00, 0x01, 0x05, 0x03, 0x00,  # B..F: unused, 1, count=5, 3, sum=0
]
cpu.load_program(program)
cpu.run(max_steps=500)
print(bits_to_int(cpu.out))   # -> 15
```

---

## Running the tests

The project ships a pytest suite covering every module, including end-to-end
instruction tests for the CPU.

```powershell
cd python
uv run pytest
```

---

## The design rules (the simulation contract)

The project follows strict rules so nothing is "cheated".

1. **Two physical states only.** A boolean models a high or low voltage.
2. **The transistor is the only primitive.** Only the `NMOS` / `PMOS` models
   may use Python logic; building the gates must not use Python's `if`,
   `and`, `or` or `not`.
3. **Assignments are wires.** `x = fn(...)` connects an output to an input;
   it is not a register.
4. **State and time.** Stateful parts (flip-flops, registers, PC) take their
   previous state as an argument and return the new one. The clock loop keeps
   the state between cycles.
5. **The clock is the external engine.** A simple Python loop drives discrete
time forward.

---

## License

This project is distributed under the
[Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0).
See the [LICENSE](LICENSE) file for the full text.
