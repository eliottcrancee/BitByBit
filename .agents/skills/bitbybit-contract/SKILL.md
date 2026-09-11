---
name: bitbybit-contract
description: The BitByBit simulation contract — the transistor-level CPU simulator in this repo (rust/ active, python/ legacy). MUST be loaded when creating, modifying, or debugging any hardware component: rust/src/hardware.rs, gates.rs, latches.rs, mux.rs, decoder.rs, arithmetic.rs, memory.rs, cpu_sap1.rs, cpu_sap2.rs, or the python/ hardware layer. Enforces transistor-only primitives, no host boolean operators in logic functions, tri-state bus resolution with short-circuit detection, clock-edge state updates, FAST skip rules, and LSB-first conventions.
---

# BitByBit contract

This is the law of the simulator. Every hardware module — from a single
transistor to the SAP-1/SAP-2 CPUs — must obey it. It is deliberately
naive: it models real digital hardware instead of using the host
language's shortcuts.

## Usage

Load and apply this skill whenever working on any component below the
assembler. That means: `rust/src/hardware.rs`, `gates.rs`, `latches.rs`,
`mux.rs`, `decoder.rs`, `arithmetic.rs`, `memory.rs`, `cpu_sap1.rs`,
`cpu_sap2.rs`, and the legacy `python/` hardware modules. If the task
touches transistors, gates, registers, buses, clocks, or control signals,
this skill applies.

## The contract

1. **Two physical states only**, plus high impedance for floating wires.
   `Bit::{Low, High}`, `Signal::{Driven(Bit), HighImpedance}`.
2. **The transistor is the only primitive.** Every logic function is a
   network of [`Pmos`] and [`Nmos`] transistors. Gates and higher
   components **never** use the host language's boolean operators
   (`&&`, `||`, `!`, bitwise ops) to compute a logic output.
3. **Assignments are wires, not registers.** `conduct_into` only
   propagates signals. Storage lives exclusively in sequential
   components, through interior mutability (`Cell<Bit>`), because
   `Component::conduct_into` takes `&self`.
4. **State and time.** Sequential parts update on clock edges
   (rising edge for master-slave flip-flops). A simple loop drives
   discrete time forward: `clock_tick` → `micro_step` →
   `step_instruction` → `run`.
5. **`FAST = true` skips provably-inactive transistors only** — the same
   nets, faster. A fast path must never change results.

## Primitive vocabulary (rust/src/hardware.rs)

- `Bit::try_from(Signal)` — reading a floating wire where a logic level
  is required fails with `HardwareError::InvalidBit`.
- `wire(signals)` — resolve one wire driven by several transistors:
  floating drivers have no effect, agreeing drivers merge, two drivers
  pulling opposite ways raise `ShortCircuit`.
- `bus8(buses)` — resolve an 8-bit bus bit per bit via `wire`.
- `stabilize(get, update, max_iterations)` — fixed-point iteration
  modeling the analog settling of a feedback loop (SR latch). A loop
  that keeps oscillating raises `UnstableCircuit`.
- `Component` trait — `const INPUTS`, `const OUTPUTS`, `conduct_into`
  (stack-sized buffers, no allocation), `transistor_count()`
  (sub-components included), plus the `conduct` and `compute` wrappers.
- `HardwareError` variants: `ShortCircuit`, `InvalidInputCount`,
  `InvalidBit`, `SrlatchForbiddenInput`, `UnstableCircuit`.

## Component hierarchy and transistor budget

Everything is CMOS, smallest composition possible:

- NOT = 1 PMOS + 1 NMOS (2T); NAND/NOR = 4T; AND = NAND + NOT (6T);
  OR = NOR + NOT (6T); XOR = 4× NAND (16T); XNOR = 18T.
- `Mux2x1` = NOT + 2 AND + OR (20T); `Mux8bits2x1` = 8× (160T).
- `HalfAdder` = XOR + AND (22T); `FullAdder` = 2× HA + OR (50T);
  `Adder8Bits` = 8× FA ripple carry.
- `SRLatch` = 2 NOR (8T); `DLatch` (22T); `DFlipFlop` master-slave (46T);
  `DFlipFlopSave` (66T); `DFlipFlopSaveLoad` (68T);
  `OneHotCounter6Bits` = 6× DFF ring (276T).
- `Decoder2to4` = 2 NOT + 4 AND; `Decoder4to16` cascades two of them.
- `Register8Bits` = 8× `DFlipFlopSaveLoad`; `Ram256Bits` = 16 registers
  + decoder + tri-state rows.

Each component must keep its documented transistor count exact, and
`transistor_count()` must sum every sub-component (add up the AND/OR
gates individually in the control unit, as `cpu_sap1.rs` does).

## Authoring a component — steps

1. Build it from existing primitives: fields are `Pmos`/`Nmos`/gates/
   latches/muxes — never raw `bool`s or `Vec<Bit>` state for storage.
2. Declare exact `INPUTS`/`OUTPUTS`. Destructure inputs with
   `let [a, b] = inputs else { return Err(HardwareError::InvalidInputCount { expected, actual }) }`
   (or the explicit length check used by `ControlUnit`).
3. Implement `conduct_into`: propagate through sub-components into
   local stack buffers, merge with `wire`/`bus8`, `?` every error.
   Propagate `HighImpedance` faithfully — never coerce it to `Low`.
4. Implement `transistor_count()` as the exact sum of all
   sub-components.
5. Add inline `#[cfg(test)]` tests: full truth table via `compute`
   (combinational) or a clock/sequence table (sequential, asserting
   updates only on the rising edge), tri-state behavior via `conduct`
   (floating outputs, `InvalidBit` on reads), forbidden inputs
   (e.g. SR both HIGH), and the exact transistor count.

## Wiring, control and bus discipline

- **Control signals are gate nets.** The control unit is AND/OR gates
  and a decoder fed by one-hot T-states, opcode and flags. A control
  line wired directly from a one-hot line (e.g. `co = t0`) is plain
  fan-out: a single driver, so **no `wire` resolution is needed**.
- **Bus drivers are tri-state NMOS rows** gated by control signals
  (`co`, `io`, `ao`, `eo`…). A disabled transistor floats its output by
  itself, so every row is propagated unconditionally and only the
  active one drives the bus. Merge with `bus8` — a short circuit
  (two active drivers disagreeing) must surface as `ShortCircuit`,
  not be silently resolved.
- **Micro-code must keep exclusive drivers.** Where exclusivity is an
  invariant, assert it (`debug_assert!(ctrl.ri == Bit::Low || ctrl.ro == Bit::Low)`).
  In SAP-2 this covers the address bus (PC/SP/MAR, with `mar_enable`
  the wired complement of PC/SP), the data-bus drivers and the RAM
  read/write pair.
- **The stack addresses the RAM directly from the SP.** Pushes
  pre-decrement the SP and write at the new address; pops read at the
  SP and then increment. Never reuse a stale MAR for a put/get: load
  the MAR only when the target must survive the push (CALL keeps the
  target in the MAR while the SP drives the stack writes).
- **Floating bits stay floating.** Sampling a floating bus bit through a
  save-enabled flip-flop raises `InvalidBit` — a loud failure instead
  of a silent stale read. Do not "fix" this by defaulting floats to LOW.
  The check applies to the **sampling** phase (clock not High): during
  the High phase the master latch is disabled, so a floating data line
  is no longer sampled and cannot latch an undefined level.

## Time and clocking

- Registers capture on the **rising edge** of `clock` when `save` is
  High; outputs float when `load` is Low (`DFlipFlopSaveLoad`).
- The sequencer (ring counter) shifts its single HIGH bit on each
  rising edge, but must still be evaluated during the LOW phase (the
  master latches sample then). `OneHotCounter6Bits` reads the last
  flip-flop *before* advancing, because the clock edge breaks the loop.
- HLT freezes the clock: once `halted`, `clock_tick` returns the same
  level without stepping.

## FAST semantics

`rust/src/lib.rs: pub const FAST: bool = true` (mirrors `bitbybit.FAST`
in python/). Fast paths may only **skip provably-inactive work**:
- SR latch with both inputs LOW returns its held state directly
  (feedback loop already settled).
- The SAP-1 ALU (`AluSap1`) with `eo` LOW floats its result **and**
  its flags, with or without `FAST`; capturing floating flags through a
  save-enabled flip-flop is then rejected — a micro-state with `fi`
  HIGH but `eo` LOW must fail loudly. (`AluSap2` exposes its flags
  unconditionally and has no such shortcut.)
- Memory receiving neither `ri` nor `ro` skips itself.
- Sequential components may skip their whole next-value tree during
  the **High** phase (adder, muxes, gating): the master latches are
  disabled then, so whatever they compute cannot reach the slave. The
  value that matters was preloaded during the Low phase. Floating
  inputs are therefore harmless during High, and only the fast path
  needs to tolerate them.

Never use FAST to change a result; if a test fails only under FAST,
the fast path is wrong. Symmetrically, a fast path must produce the
same outputs as the full path for every driven input: the full path
may compute more, but the observable result must match.

## Conventions

- Multi-bit values are **LSB-first** everywhere (arrays and
  `bits_to_int`/`int_to_bits` in `utils.rs`). Byte↔bit conversion is
  not electrical behavior — it lives in `utils.rs`, not hardware.
- Combinational use `compute(&[Bit; ...])` (Bits in, every output must
  be driven). Sequential and tri-state behavior uses `conduct(&[Signal; ...])`.
- Errors propagate with `?`. `expect` is allowed only where failure is
  provably impossible (driven inputs into a NOR, etc.); `assert` only
  for programmer errors (e.g. `stabilize` with zero iterations).
- Pattern matching on `Bit`/`Signal` is fine (it selects, it does not
  compute a logic function). Equality comparisons are allowed for input
  validation and FAST condition checks only.

## Verification

Run from `rust/`:

```powershell
cargo test          # 183 tests: every module + end-to-end programs
cargo run --release # SAP-1 benchmark assembled from text
```

After any component change: run `cargo test`; add at least one truth
table, one tri-state/error, and one transistor-count test per new
component. End-to-end program tests live in `cpu_sap1.rs` /
`cpu_sap2.rs` — a changed datapath must still run `LDI 5, ADD 0xE, HLT`
and the loop-multiplication program with identical results.

## Checklist before submitting a component

- [ ] No host boolean/bitwise operator computes any logic output.
- [ ] All storage uses latches/flip-flops, not plain variables.
- [ ] `INPUTS`/`OUTPUTS` exact; wrong counts (inputs **and** outputs)
  raise `InvalidInputCount`, never a slice panic.
- [ ] High impedance propagates; reads of floating wires raise `InvalidBit`.
- [ ] `transistor_count()` matches the documented budget exactly.
- [ ] FAST path exists only where a component can prove itself inactive.
- [ ] Tests: truth table, tri-state/errors, sequence/edges, transistor count.
- [ ] `cargo test` green.
