//! Storage components layered on top of [`crate::latches`],
//! [`crate::arithmetic`], [`crate::mux`] and [`crate::decoder`].
//!
//! All multi-bit values are LSB-first, matching the convention of
//! [`crate::hardware`].
//!
//!     Register8Bits
//!         Eight D flip-flops with save/load. Writes on the rising edge
//!         of `clock` when `save` is High; outputs float when `load`
//!         is Low.
//!
//!     ProgramCounter4Bits
//!         Increments itself on each rising clock edge, or loads
//!         `data` instead when `save` is High. Only the four low bits
//!         are exposed; the value wraps back to zero on overflow.
//!
//!     Ram256Bits
//!         Sixteen 8-bit registers addressed by a 4-to-16 decoder. Only
//!         the addressed register drives the output bus on load.
//!
//!     Ram64KBits
//!         65536 bytes built by cascading 4096 Ram256Bits chips. The
//!         three high address nibbles select one chip through cascaded
//!         decoders and AND gates; the low nibble addresses the 16
//!         registers inside every chip.
//!
//!     FlagsRegister
//!         Three independent flag latches (zero, carry, sign) written
//!         together on a common save pulse; outputs always driven.
//!
//!     ProgramCounter16Bits
//!         16-bit counter built from two 8-bit registers and a 16-bit
//!         half-adder incrementer. Counts when `count` is High, loads
//!         `data` into either half when the matching save line is
//!         High, outputs gated by `load`.
//!
//!     StackPointer16Bits
//!         16-bit up/down counter built from two 8-bit registers and
//!         a 16-bit full-adder chain: one adder input per bit is the
//!         direction line (Low = increment, High = decrement) and the
//!         initial carry is its complement.
//!
//!     InputPort
//!         Eight tri-state switches driving the data bus on demand.
//!
//!     OutputRegister
//!         An 8-bit latch holding the value driven by OUT; read
//!         outside the CPU through its always-driven outputs.
//!
//!     MemoryAddressRegister16Bits
//!         Two 8-bit registers loaded in two halves from the 8-bit
//!         bus; the 16-bit address drives the address bus ungated.

use crate::arithmetic::{Adder8Bits, FullAdder, HalfAdder};
use crate::decoder::Decoder4to16;
use crate::gates::{AndGate, NotGate, OrGate};
use crate::hardware::{bus8, Bit, Component, HardwareError, Nmos, Signal};
use crate::latches::DFlipFlopSaveLoad;
use crate::mux::Mux8bits2x1;
use crate::FAST;

pub const ADDRESS_BITS: usize = 4;
pub const RAM_REGISTERS: usize = 16;

const ZERO_BYTE: [Signal; 8] = [Signal::Driven(Bit::Low); 8];
const ONE_BYTE: [Signal; 8] = [
    Signal::Driven(Bit::High),
    Signal::Driven(Bit::Low),
    Signal::Driven(Bit::Low),
    Signal::Driven(Bit::Low),
    Signal::Driven(Bit::Low),
    Signal::Driven(Bit::Low),
    Signal::Driven(Bit::Low),
    Signal::Driven(Bit::Low),
];

/// Eight-bit register: writes on the rising edge when `save` is High.
#[derive(Debug, Default)]
pub struct Register8Bits {
    flip_flops: [DFlipFlopSaveLoad; 8],
}

impl Register8Bits {
    /// The stored byte, LSB first.
    pub fn state(&self) -> [Bit; 8] {
        self.flip_flops.each_ref().map(|flip_flop| flip_flop.q())
    }
}

impl Component for Register8Bits {
    /// Inputs:
    ///
    /// - 0..8:  data, least significant bit first
    /// - 8:     clock
    /// - 9:     save
    /// - 10:    load
    ///
    /// Outputs:
    ///
    /// - 0..8: the stored byte, gated by `load`. Every output floats
    ///   when `load` is Low.
    ///
    /// Store `data` on the rising edge of `clock` when `save` is High.
    const INPUTS: usize = 11;
    const OUTPUTS: usize = 8;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let clock = inputs[8];
        let save = inputs[9];
        let load = inputs[10];

        // With `FAST`, a High clock disables the master latches and a
        // Low `save` blocks the slave capture: no flip-flop can change
        // state during this call, so the whole propagation is skipped
        // and the stored state is returned (floating when `load` is
        // Low). Low-clock calls are always propagated: the masters
        // must preload there, since they are the value captured at the
        // next rising edge.
        if FAST && clock == Signal::Driven(Bit::High) && save == Signal::Driven(Bit::Low) {
            match load {
                Signal::Driven(Bit::High) => {
                    for (slot, bit) in outputs.iter_mut().zip(self.state()) {
                        *slot = Signal::from(bit);
                    }
                }
                _ => outputs.fill(Signal::HighImpedance),
            }

            return Ok(());
        }

        let mut slot = [Signal::HighImpedance; DFlipFlopSaveLoad::OUTPUTS];

        for (index, flip_flop) in self.flip_flops.iter().enumerate() {
            flip_flop.conduct_into(&[inputs[index], clock, save, load], &mut slot)?;
            outputs[index] = slot[0];
        }

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.flip_flops
            .iter()
            .map(|flip_flop| flip_flop.transistor_count())
            .sum()
    }
}

/// Four-bit program counter: increments, or loads on the rising edge.
#[derive(Debug, Default)]
pub struct ProgramCounter4Bits {
    register: Register8Bits,
    adder: Adder8Bits,
    load_mux: Mux8bits2x1,
    overflow_mux: Mux8bits2x1,
    output_nmos: [Nmos; ADDRESS_BITS],
}

impl ProgramCounter4Bits {
    /// The full byte stored by the inner register.
    pub fn state(&self) -> [Bit; 8] {
        self.register.state()
    }
}

impl ProgramCounter4Bits {
    /// Gate the four low bits of `stored` through the output NMOS row.
    fn gate_output(
        &self,
        load: Signal,
        stored: &[Signal; 8],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        for (index, nmos) in self.output_nmos.iter().enumerate() {
            nmos.conduct_into(&[load, stored[index]], &mut outputs[index..index + 1])?;
        }

        Ok(())
    }
}

impl Component for ProgramCounter4Bits {
    /// Inputs:
    ///
    /// - 0..4: data, least significant bit first
    /// - 4:    clock
    /// - 5:    save
    /// - 6:    load
    ///
    /// Outputs:
    ///
    /// - 0..4: the four low bits, gated by `load`.
    ///
    /// Increment, or load `data` when `save` is High.
    ///
    /// The next value is computed combinationally from the stored one:
    /// `state + 1` is selected against `data` by `save`, and the result
    /// wraps to zero when the fifth bit overflows. The value is written
    /// on the rising edge, then the four low bits are gated by `load`.
    ///
    /// The internal register receives `save = clock`: combined with the
    /// already gated `clock`, the counter advances exactly on its clock
    /// rising edges. The feedback path goes through the register's
    /// flip-flops, so a single propagation pass computes the new state.
    const INPUTS: usize = 7;
    const OUTPUTS: usize = 4;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let clock = inputs[4];
        let save = inputs[5];
        let load = inputs[6];

        // With `FAST`, a High clock disables the register's master
        // latches: the slaves capture the value preloaded during the
        // Low tick and the data inputs are ignored. The adder and
        // both multiplexers are therefore skipped and the register
        // receives dummy driven data. Low-clock calls always run the
        // full combinational path, since the masters must preload
        // `state + 1` (or the load data) there.
        if FAST && clock == Signal::Driven(Bit::High) {
            let mut register_inputs = [Signal::Driven(Bit::Low); 11];
            register_inputs[8] = clock;
            register_inputs[9] = clock;
            register_inputs[10] = Signal::Driven(Bit::High);

            let mut stored = [Signal::HighImpedance; 8];
            self.register.conduct_into(&register_inputs, &mut stored)?;

            return self.gate_output(load, &stored, outputs);
        }

        // incremented = state + 1, LSB first. The adder also returns a
        // carry bit, which is not needed here.
        let state = self.register.state();
        let mut adder_inputs = [Signal::HighImpedance; 17];

        for (slot, bit) in adder_inputs[..8].iter_mut().zip(state.iter()) {
            *slot = Signal::from(*bit);
        }

        adder_inputs[8..16].copy_from_slice(&ONE_BYTE);
        adder_inputs[16] = Signal::Driven(Bit::Low);

        let mut incremented = [Signal::HighImpedance; 9];
        self.adder.conduct_into(&adder_inputs, &mut incremented)?;

        // selected = save ? data : state + 1
        //
        // `data` must be zero-extended to 8 bits for the multiplexer.
        let mut load_inputs = [Signal::HighImpedance; 17];
        load_inputs[..8].copy_from_slice(&incremented[..8]);
        load_inputs[8..12].copy_from_slice(&inputs[0..4]);
        load_inputs[12..16].copy_from_slice(&ZERO_BYTE[..4]);
        load_inputs[16] = save;

        let mut selected = [Signal::HighImpedance; 8];
        self.load_mux.conduct_into(&load_inputs, &mut selected)?;

        // wrapped = selected[4] ? 0 : selected
        //
        // A 4-bit counter only exposes its low bits; counting past 15
        // sets the fifth bit, which resets the stored value to zero.
        let mut overflow_inputs = [Signal::HighImpedance; 17];
        overflow_inputs[..8].copy_from_slice(&selected);
        overflow_inputs[8..16].copy_from_slice(&ZERO_BYTE);
        overflow_inputs[16] = selected[4];

        let mut wrapped = [Signal::HighImpedance; 8];
        self.overflow_mux
            .conduct_into(&overflow_inputs, &mut wrapped)?;

        // Store `wrapped` on the rising edge; the register outputs are
        // always driven, the external `load` gates them afterwards.
        let mut register_inputs = [Signal::HighImpedance; 11];
        register_inputs[..8].copy_from_slice(&wrapped);
        register_inputs[8] = clock;
        register_inputs[9] = clock;
        register_inputs[10] = Signal::Driven(Bit::High);

        let mut stored = [Signal::HighImpedance; 8];
        self.register.conduct_into(&register_inputs, &mut stored)?;

        self.gate_output(load, &stored, outputs)
    }

    fn transistor_count(&self) -> usize {
        self.register.transistor_count()
            + self.adder.transistor_count()
            + self.load_mux.transistor_count()
            + self.overflow_mux.transistor_count()
            + self
                .output_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
    }
}

/// 256-bit RAM: 16 registers of 8 bits selected by a 4-bit address.
#[derive(Debug, Default)]
pub struct Ram256Bits {
    registers: [Register8Bits; RAM_REGISTERS],
    decoder: Decoder4to16,
    save_gates: [AndGate; RAM_REGISTERS],
    load_gates: [AndGate; RAM_REGISTERS],
}

impl Ram256Bits {
    /// The stored content of every register, in address order.
    pub fn state(&self) -> [[Bit; 8]; RAM_REGISTERS] {
        self.registers.each_ref().map(|register| register.state())
    }

    /// Force-write `data` at `address`, outside any CPU clock cycle.
    ///
    /// This emulates the LOW->HIGH->LOW clock pulse needed by the
    /// register's flip-flops to capture `data`. It is meant for loading
    /// a program before execution starts.
    ///
    /// `address` must be smaller than [`RAM_REGISTERS`].
    pub fn write_byte(&self, address: usize, data: [Bit; 8]) -> Result<(), HardwareError> {
        assert!(
            address < RAM_REGISTERS,
            "address must be within [0, {}].",
            RAM_REGISTERS - 1
        );

        let register = &self.registers[address];

        let mut inputs = [Signal::HighImpedance; 11];

        for (slot, bit) in inputs[..8].iter_mut().zip(data.iter()) {
            *slot = Signal::from(*bit);
        }

        inputs[10] = Signal::Driven(Bit::Low);

        let mut stored = [Signal::HighImpedance; 8];

        // Master latch receives the data while the clock is LOW, the
        // slave latch updates during HIGH, then the clock returns LOW.
        inputs[8] = Signal::from(Bit::Low);
        inputs[9] = Signal::from(Bit::High);
        register.conduct_into(&inputs, &mut stored)?;

        inputs[8] = Signal::from(Bit::High);
        register.conduct_into(&inputs, &mut stored)?;

        inputs[8] = Signal::from(Bit::Low);
        inputs[9] = Signal::from(Bit::Low);
        register.conduct_into(&inputs, &mut stored)?;

        Ok(())
    }
}

impl Component for Ram256Bits {
    /// Inputs:
    ///
    /// - 0..8:   data, least significant bit first
    /// - 8..12:  address, least significant bit first
    /// - 12:     clock
    /// - 13:     save
    /// - 14:     load
    ///
    /// Outputs:
    ///
    /// - 0..8: the addressed byte, gated by `load`. Every output floats
    ///   when `load` is Low.
    ///
    /// Write `data` at `address` on the rising edge, or read it.
    ///
    /// The decoder activates a single register. The `save` and `load`
    /// signals are ANDed with that selection, so only the addressed
    /// register can be written or can drive the output bus. Registers
    /// that receive neither `save` nor `load` are skipped entirely:
    /// they cannot change state nor drive the bus.
    const INPUTS: usize = 15;
    const OUTPUTS: usize = 8;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let data = &inputs[0..8];
        let address = &inputs[8..12];
        let clock = inputs[12];
        let save = inputs[13];
        let load = inputs[14];

        // With `FAST`, a memory receiving neither `save` nor `load`
        // can neither change state nor drive the bus: skip the
        // decoder, the gating gates and every register entirely.
        if FAST && save == Signal::Driven(Bit::Low) && load == Signal::Driven(Bit::Low) {
            outputs.fill(Signal::HighImpedance);
            return Ok(());
        }

        let mut selected = [Signal::HighImpedance; 16];
        self.decoder.conduct_into(address, &mut selected)?;

        let mut drivers = [[Signal::HighImpedance; 8]; RAM_REGISTERS];

        for (index, select_line) in selected.iter().enumerate() {
            let mut gate_out = [Signal::HighImpedance; 1];
            self.save_gates[index].conduct_into(&[*select_line, save], &mut gate_out)?;
            let register_save = gate_out[0];
            self.load_gates[index].conduct_into(&[*select_line, load], &mut gate_out)?;
            let register_load = gate_out[0];

            // Unselected registers skip themselves inside
            // `Register8Bits` (their gated `save` and `load` are both
            // Low): they return a floating row without any flip-flop
            // propagation.
            let mut register_inputs = [Signal::HighImpedance; 11];
            register_inputs[..8].copy_from_slice(data);
            register_inputs[8] = clock;
            register_inputs[9] = register_save;
            register_inputs[10] = register_load;

            self.registers[index].conduct_into(&register_inputs, &mut drivers[index])?;
        }

        outputs.copy_from_slice(&bus8(drivers)?);

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.registers
            .iter()
            .map(|register| register.transistor_count())
            .sum::<usize>()
            + self.decoder.transistor_count()
            + self
                .save_gates
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
            + self
                .load_gates
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
    }
}

/// Number of address bits of a [`Ram64KBits`]: a 16-bit address space.
pub const RAM64K_ADDRESS_BITS: usize = 16;

/// Number of [`Ram256Bits`] chips cascaded into a [`Ram64KBits`]:
/// 4096 chips x 16 registers x 8 bits = 65536 bytes.
pub const RAM64K_CHIPS: usize = 4096;

/// Number of bank lines: the two highest nibbles select one of 256
/// banks, the third nibble selects a chip inside the bank.
const RAM64K_BANKS: usize = 256;

/// 64 KiB RAM built by cascading 4096 [`Ram256Bits`] chips.
///
/// The 16-bit address (least significant bit first) is split in four
/// nibbles:
///
/// - `address[0..4]` selects one of the 16 registers *inside* every
///   chip, through each chip's internal decoder;
/// - `address[4..8]` selects one of the 16 chips in a bank;
/// - `address[8..12]` and `address[12..16]` select one of the 256
///   banks, through two cascaded decoders whose outputs are ANDed
///   pairwise.
///
/// The resulting chip-select net gates the external `save` and `load`
/// signals, so only the selected chip can be written or can drive the
/// output bus — exactly like a real board where the high address bits
/// drive the chips' chip-enable pins. With `FAST`, an idle RAM (both
/// `save` and `load` Low) skips the whole select tree, and every
/// unselected chip skips its own propagation.
impl Default for Ram64KBits {
    fn default() -> Self {
        Self {
            chips: (0..RAM64K_CHIPS)
                .map(|_| Ram256Bits::default())
                .collect::<Vec<_>>()
                .into_boxed_slice()
                .try_into()
                .expect("exactly RAM64K_CHIPS chips"),
            bank_ands: std::array::from_fn(|_| AndGate::default()),
            select_ands: std::array::from_fn(|_| AndGate::default()),
            save_gates: std::array::from_fn(|_| AndGate::default()),
            load_gates: std::array::from_fn(|_| AndGate::default()),
            group_decoder: Decoder4to16::default(),
            bank_low_decoder: Decoder4to16::default(),
            bank_high_decoder: Decoder4to16::default(),
        }
    }
}

#[derive(Debug)]
pub struct Ram64KBits {
    /// Boxed: 4096 registers are far too large for the caller's stack.
    chips: Box<[Ram256Bits; RAM64K_CHIPS]>,
    /// `bank_ands[bank]` = high_bank[bank / 16] AND low_bank[bank % 16].
    bank_ands: [AndGate; RAM64K_BANKS],
    /// `select_ands[chip]` = bank line AND group line (chip select).
    select_ands: [AndGate; RAM64K_CHIPS],
    /// `save_gates[chip]` = chip select AND external `save`.
    save_gates: [AndGate; RAM64K_CHIPS],
    /// `load_gates[chip]` = chip select AND external `load`.
    load_gates: [AndGate; RAM64K_CHIPS],
    /// Decodes `address[4..8]`: which chip in the bank.
    group_decoder: Decoder4to16,
    /// Decodes `address[8..12]`: low part of the bank number.
    bank_low_decoder: Decoder4to16,
    /// Decodes `address[12..16]`: high part of the bank number.
    bank_high_decoder: Decoder4to16,
}

impl Ram64KBits {
    /// The stored byte at `address`.
    pub fn state(&self, address: usize) -> [Bit; 8] {
        self.chips[address >> 4].state()[address & 0xF]
    }

    /// Force-write `data` at `address`, outside any CPU clock cycle.
    ///
    /// This emulates the LOW->HIGH->LOW clock pulse needed by the
    /// register's flip-flops to capture `data`. It is meant for loading
    /// a program before execution starts.
    ///
    /// `address` must be smaller than `1 << RAM64K_ADDRESS_BITS`.
    pub fn write_byte(&self, address: usize, data: [Bit; 8]) -> Result<(), HardwareError> {
        assert!(
            address < 1 << RAM64K_ADDRESS_BITS,
            "address must be within [0, {}].",
            (1 << RAM64K_ADDRESS_BITS) - 1
        );

        let chip_index = address >> 4;
        let register_index = address & 0xF;

        self.chips[chip_index].write_byte(register_index, data)
    }
}

impl Component for Ram64KBits {
    /// Inputs:
    ///
    /// - 0..8:   data, least significant bit first
    /// - 8..24:  address, least significant bit first
    /// - 24:     clock
    /// - 25:     save
    /// - 26:     load
    ///
    /// Outputs:
    ///
    /// - 0..8: the addressed byte, gated by `load`. Every output floats
    ///   when `load` is Low.
    ///
    /// Write `data` at `address` on the rising edge, or read it.
    ///
    /// Three cascaded decoders and two AND levels select exactly one
    /// chip; the chip's internal decoder then selects exactly one of
    /// its 16 registers. Only the selected chip receives the gated
    /// `save` / `load` signals, so all the other chips float their
    /// output row without any flip-flop propagation.
    const INPUTS: usize = 8 + RAM64K_ADDRESS_BITS + 3;
    const OUTPUTS: usize = 8;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let data = &inputs[0..8];
        let address = &inputs[8..24];
        let clock = inputs[24];
        let save = inputs[25];
        let load = inputs[26];

        // With `FAST`, a memory receiving neither `save` nor `load`
        // can neither change state nor drive the bus: skip the select
        // tree and every chip entirely.
        if FAST && save == Signal::Driven(Bit::Low) && load == Signal::Driven(Bit::Low) {
            outputs.fill(Signal::HighImpedance);
            return Ok(());
        }

        let mut group_lines = [Signal::HighImpedance; 16];
        self.group_decoder
            .conduct_into(&address[4..8], &mut group_lines)?;

        let mut bank_low_lines = [Signal::HighImpedance; 16];
        self.bank_low_decoder
            .conduct_into(&address[8..12], &mut bank_low_lines)?;

        let mut bank_high_lines = [Signal::HighImpedance; 16];
        self.bank_high_decoder
            .conduct_into(&address[12..16], &mut bank_high_lines)?;

        // bank = low + 16 * high, matching the chip index
        // `group + 16 * low + 256 * high` used below.
        let mut bank_lines = [Signal::HighImpedance; RAM64K_BANKS];
        let mut single = [Signal::HighImpedance; 1];

        for (bank, line) in bank_lines.iter_mut().enumerate() {
            self.bank_ands[bank].conduct_into(
                &[bank_high_lines[bank / 16], bank_low_lines[bank % 16]],
                &mut single,
            )?;
            *line = single[0];
        }

        let mut drivers = [[Signal::HighImpedance; 8]; RAM64K_CHIPS];

        for (chip_index, row) in drivers.iter_mut().enumerate() {
            self.select_ands[chip_index].conduct_into(
                &[bank_lines[chip_index / 16], group_lines[chip_index % 16]],
                &mut single,
            )?;
            let chip_select = single[0];

            self.save_gates[chip_index].conduct_into(&[chip_select, save], &mut single)?;
            let chip_save = single[0];
            self.load_gates[chip_index].conduct_into(&[chip_select, load], &mut single)?;
            let chip_load = single[0];

            // Unselected chips skip themselves inside `Ram256Bits`
            // (their gated `save` and `load` are both Low): they
            // return a floating row without any register propagation.
            // The explicit check avoids even building their inputs.
            if FAST
                && chip_save == Signal::Driven(Bit::Low)
                && chip_load == Signal::Driven(Bit::Low)
            {
                continue;
            }

            let mut chip_inputs = [Signal::HighImpedance; 15];
            chip_inputs[..8].copy_from_slice(data);
            chip_inputs[8..12].copy_from_slice(&address[0..4]);
            chip_inputs[12] = clock;
            chip_inputs[13] = chip_save;
            chip_inputs[14] = chip_load;

            self.chips[chip_index].conduct_into(&chip_inputs, row)?;
        }

        outputs.copy_from_slice(&bus8(drivers)?);

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.chips
            .iter()
            .map(|chip| chip.transistor_count())
            .sum::<usize>()
            + self.group_decoder.transistor_count()
            + self.bank_low_decoder.transistor_count()
            + self.bank_high_decoder.transistor_count()
            + self
                .bank_ands
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
            + self
                .select_ands
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
            + self
                .save_gates
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
            + self
                .load_gates
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
    }
}

/// Three flag latches for the SAP-2: zero, carry and sign.
///
/// All three capture their input bit on the rising edge of `clock`
/// when `save` is High — typically the control unit's `fi` pulse at
/// the end of an ALU operation. The outputs are always driven: the
/// control unit reads the flags directly, not through the shared bus.
#[derive(Debug, Default)]
pub struct FlagsRegister {
    zero: DFlipFlopSaveLoad,
    carry: DFlipFlopSaveLoad,
    sign: DFlipFlopSaveLoad,
}

impl FlagsRegister {
    /// The stored flags as `(zero, carry, sign)`.
    pub fn state(&self) -> (Bit, Bit, Bit) {
        (self.zero.q(), self.carry.q(), self.sign.q())
    }
}

impl Component for FlagsRegister {
    /// Inputs:
    ///
    /// - 0: zero flag input
    /// - 1: carry flag input
    /// - 2: sign flag input
    /// - 3: clock
    /// - 4: save
    ///
    /// Outputs:
    ///
    /// - 0: zero flag output
    /// - 1: carry flag output
    /// - 2: sign flag output
    const INPUTS: usize = 5;
    const OUTPUTS: usize = 3;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let clock = inputs[3];
        let save = inputs[4];

        // The flag outputs drive internal nets, not the shared bus:
        // each flip-flop's `load` pin is tied HIGH. The flip-flop has
        // two outputs (q, q_bar); only q is exposed.
        let tied_high = Signal::Driven(Bit::High);
        let mut both = [Signal::HighImpedance; 2];

        for (index, flip_flop) in [&self.zero, &self.carry, &self.sign]
            .into_iter()
            .enumerate()
        {
            flip_flop.conduct_into(&[inputs[index], clock, save, tied_high], &mut both)?;
            outputs[index] = both[0];
        }

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.zero.transistor_count() + self.carry.transistor_count() + self.sign.transistor_count()
    }
}

/// 16-bit memory address register feeding the tri-state address bus.
///
/// Three load paths per half, in decreasing priority:
///
/// - `save_low` / `save_high`: capture the 8-bit data bus into one
///   half (operand address bytes of LDA/STA/JMP/CALL/RET);
/// - `load16`: capture the full 16-bit address bus at once (fetch,
///   when the PC drives it; stack access, when the SP drives it);
/// - otherwise hold.
///
/// The stored address drives the address bus through 16 tri-state
/// drivers gated by `enable` — the PC and the SP share the bus, so
/// the control unit never asserts two address-bus drivers at once.
#[derive(Debug, Default)]
pub struct MemoryAddressRegister16Bits {
    registers: [Register8Bits; 2],
    /// Per half: hold or data-bus byte, on the half's `save` line.
    bus_muxes: [Mux8bits2x1; 2],
    /// Per half: bus byte or address-bus half, on `load16`.
    addr_muxes: [Mux8bits2x1; 2],
    output_nmos: [Nmos; 16],
}

impl MemoryAddressRegister16Bits {
    /// The stored address, least significant bit first.
    pub fn state(&self) -> [Bit; 16] {
        let low = self.registers[0].state();
        let high = self.registers[1].state();

        let mut address = [Bit::Low; 16];
        address[..8].copy_from_slice(&low);
        address[8..].copy_from_slice(&high);
        address
    }

    /// Gate the stored address through the output NMOS row.
    fn gate_output(&self, enable: Signal, outputs: &mut [Signal]) -> Result<(), HardwareError> {
        let stored = self.state();

        for (index, nmos) in self.output_nmos.iter().enumerate() {
            nmos.conduct_into(
                &[enable, Signal::from(stored[index])],
                &mut outputs[index..index + 1],
            )?;
        }

        Ok(())
    }
}

impl Component for MemoryAddressRegister16Bits {
    /// Inputs:
    ///
    /// - 0..8:  data, least significant bit first (the shared data bus)
    /// - 8..24: address, least significant bit first (the shared
    ///   address bus)
    /// - 24:    clock
    /// - 25:    save_low — capture `data` into the low byte
    /// - 26:    save_high — capture `data` into the high byte
    /// - 27:    load16 — capture the full address bus
    /// - 28:    enable — gate the stored address onto the address bus
    ///
    /// Outputs:
    ///
    /// - 0..16: the stored address, gated by `enable`.
    const INPUTS: usize = 8 + 16 + 5;
    const OUTPUTS: usize = 16;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let clock = inputs[24];
        let saves = [inputs[25], inputs[26]];
        let load16 = inputs[27];
        let enable = inputs[28];

        // With `FAST`, a High clock means the registers' master
        // latches are disabled: the next value was preloaded during
        // the Low tick, so the mux tree can be skipped entirely.
        if FAST && clock == Signal::Driven(Bit::High) {
            let mut register_inputs = [Signal::Driven(Bit::Low); 11];
            register_inputs[8] = clock;
            register_inputs[9] = clock;
            register_inputs[10] = Signal::Driven(Bit::High);

            let mut unused = [Signal::HighImpedance; 8];
            for register in &self.registers {
                register.conduct_into(&register_inputs, &mut unused)?;
            }

            return self.gate_output(enable, outputs);
        }

        let stored = self.state();
        let mut state = [Signal::HighImpedance; 16];
        for (index, bit) in stored.iter().enumerate() {
            state[index] = Signal::from(*bit);
        }

        let mut mid = [Signal::HighImpedance; 16];
        let mut next = [Signal::HighImpedance; 16];
        let mut mux_inputs = [Signal::HighImpedance; 17];

        for (half, register) in self.registers.iter().enumerate() {
            let base = half * 8;

            // Data bus byte into one half.
            mux_inputs[..8].copy_from_slice(&state[base..base + 8]);
            mux_inputs[8..16].copy_from_slice(&inputs[0..8]);
            mux_inputs[16] = saves[half];
            self.bus_muxes[half].conduct_into(&mux_inputs, &mut mid[base..base + 8])?;

            // Address bus half (16-bit load).
            mux_inputs[..8].copy_from_slice(&mid[base..base + 8]);
            mux_inputs[8..16].copy_from_slice(&inputs[8 + base..16 + base]);
            mux_inputs[16] = load16;
            self.addr_muxes[half].conduct_into(&mux_inputs, &mut next[base..base + 8])?;

            let mut register_inputs = [Signal::HighImpedance; 11];
            register_inputs[..8].copy_from_slice(&next[base..base + 8]);
            register_inputs[8] = clock;
            register_inputs[9] = clock;
            register_inputs[10] = Signal::Driven(Bit::High);

            let mut unused = [Signal::HighImpedance; 8];
            register.conduct_into(&register_inputs, &mut unused)?;
        }

        self.gate_output(enable, outputs)
    }

    fn transistor_count(&self) -> usize {
        self.registers
            .iter()
            .map(|register| register.transistor_count())
            .sum::<usize>()
            + self
                .bus_muxes
                .iter()
                .map(|mux| mux.transistor_count())
                .sum::<usize>()
            + self
                .addr_muxes
                .iter()
                .map(|mux| mux.transistor_count())
                .sum::<usize>()
            + self
                .output_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
    }
}

/// 16-bit program counter with three load paths and three output
/// paths, sharing the tri-state address and data buses:
///
/// Loads (decreasing priority, per half):
///
/// - `save_low` / `save_high`: capture the 8-bit data bus (RET pops
///   the return address byte by byte);
/// - `load_addr`: capture the 16-bit address bus (JMP/JZ/JNZ/JM/CALL:
///   the MAR is driving it with the target);
/// - `count`: increment by one (instruction fetch).
///
/// Outputs:
///
/// - the full 16-bit value onto the address bus, gated by `enable`
///   (instruction fetch and operand addressing);
/// - the low byte or the high byte onto the data bus, gated by
///   `out_lo` / `out_hi` (CALL pushes the return address).
#[derive(Debug, Default)]
pub struct ProgramCounter16Bits {
    registers: [Register8Bits; 2],
    /// Half-adder chain computing `state + 1` over the 16 bits.
    half_adders: [HalfAdder; 16],
    /// `count_muxes[half]`: state when `count` Low, incremented when High.
    count_muxes: [Mux8bits2x1; 2],
    /// `addr_muxes[half]`: counter value or address-bus half.
    addr_muxes: [Mux8bits2x1; 2],
    /// `data_muxes[half]`: previous value or data-bus byte.
    data_muxes: [Mux8bits2x1; 2],
    /// Address-bus drivers, one bit each, gated by `enable`.
    address_nmos: [Nmos; 16],
    /// Data-bus drivers: the two bytes, gated by `out_lo` / `out_hi`.
    data_nmos: [Nmos; 16],
}

impl ProgramCounter16Bits {
    /// The stored 16-bit value, least significant bit first.
    pub fn state(&self) -> [Bit; 16] {
        let low = self.registers[0].state();
        let high = self.registers[1].state();

        let mut value = [Bit::Low; 16];
        value[..8].copy_from_slice(&low);
        value[8..].copy_from_slice(&high);
        value
    }

    /// Gate the stored 16 bits onto the address bus.
    fn gate_address(&self, enable: Signal, outputs: &mut [Signal]) -> Result<(), HardwareError> {
        let stored = self.state();

        for (index, nmos) in self.address_nmos.iter().enumerate() {
            nmos.conduct_into(
                &[enable, Signal::from(stored[index])],
                &mut outputs[index..index + 1],
            )?;
        }

        Ok(())
    }

    /// Drive the data-bus byte: `out_lo` selects the low byte,
    /// `out_hi` the high byte (never both in a valid micro-code).
    fn gate_data(&self, out_lo: Signal, out_hi: Signal, outputs: &mut [Signal]) {
        let stored = self.state();
        let enables = [out_lo, out_hi];
        let mut drivers = [[Signal::HighImpedance; 8]; 2];

        for (group, enable) in enables.iter().enumerate() {
            for index in 0..8 {
                self.data_nmos[8 * group + index]
                    .conduct_into(
                        &[*enable, Signal::from(stored[8 * group + index])],
                        &mut drivers[group][index..index + 1],
                    )
                    .expect("data driver failed");
            }
        }

        let resolved = bus8(drivers).expect("PC data byte conflict");

        for (slot, signal) in outputs[16..24].iter_mut().zip(resolved.iter()) {
            *slot = *signal;
        }
    }
}

impl Component for ProgramCounter16Bits {
    /// Inputs:
    ///
    /// - 0..8:  data, least significant bit first (the shared data bus)
    /// - 8..24: address, least significant bit first (the shared
    ///   address bus)
    /// - 24:    clock
    /// - 25:    count — increment on the rising edge when High
    /// - 26:    load_addr — capture the full address bus
    /// - 27:    save_low — capture `data` into the low byte
    /// - 28:    save_high — capture `data` into the high byte
    /// - 29:    enable — gate the 16-bit value onto the address bus
    /// - 30:    out_lo — gate the low byte onto the data bus
    /// - 31:    out_hi — gate the high byte onto the data bus
    ///
    /// Outputs:
    ///
    /// - 0..16:  the counter value, gated by `enable` (address bus)
    /// - 16..24: the selected byte, gated by `out_lo` / `out_hi`
    const INPUTS: usize = 8 + 16 + 8;
    const OUTPUTS: usize = 24;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let clock = inputs[24];
        let count = inputs[25];
        let load_addr = inputs[26];
        let saves = [inputs[27], inputs[28]];
        let enable = inputs[29];
        let out_lo = inputs[30];
        let out_hi = inputs[31];

        // With `FAST`, a High clock disables the registers' master
        // latches: the next value was preloaded during the Low tick
        // and the incrementer and all six multiplexers are skipped.
        if FAST && clock == Signal::Driven(Bit::High) {
            let mut register_inputs = [Signal::Driven(Bit::Low); 11];
            register_inputs[8] = clock;
            register_inputs[9] = clock;
            register_inputs[10] = Signal::Driven(Bit::High);

            let mut unused = [Signal::HighImpedance; 8];
            for register in &self.registers {
                register.conduct_into(&register_inputs, &mut unused)?;
            }

            self.gate_address(enable, outputs)?;
            self.gate_data(out_lo, out_hi, outputs);
            return Ok(());
        }

        // Incremented = state + 1 over the full 16 bits, LSB first:
        // a ripple of half adders, carry starting at Low.
        let mut state = [Signal::HighImpedance; 16];
        for (index, bit) in self.state().iter().enumerate() {
            state[index] = Signal::from(*bit);
        }

        let mut incremented = [Signal::HighImpedance; 16];
        // Incrementing by one means adding 1: the ripple starts with
        // a carry-in of High.
        let mut carry = Signal::Driven(Bit::High);
        let mut ha_out = [Signal::HighImpedance; 2];

        for index in 0..16 {
            self.half_adders[index].conduct_into(&[state[index], carry], &mut ha_out)?;
            incremented[index] = ha_out[0];
            carry = ha_out[1];
        }

        // Per half: count mux, then address-bus load, then data-bus
        // byte load (highest priority).
        let mut counted = [Signal::HighImpedance; 16];
        let mut addressed = [Signal::HighImpedance; 16];
        let mut next = [Signal::HighImpedance; 16];
        let mut mux_inputs = [Signal::HighImpedance; 17];

        for (half, register) in self.registers.iter().enumerate() {
            let base = half * 8;

            mux_inputs[..8].copy_from_slice(&state[base..base + 8]);
            mux_inputs[8..16].copy_from_slice(&incremented[base..base + 8]);
            mux_inputs[16] = count;
            self.count_muxes[half].conduct_into(&mux_inputs, &mut counted[base..base + 8])?;

            mux_inputs[..8].copy_from_slice(&counted[base..base + 8]);
            mux_inputs[8..16].copy_from_slice(&inputs[8 + base..16 + base]);
            mux_inputs[16] = load_addr;
            self.addr_muxes[half].conduct_into(&mux_inputs, &mut addressed[base..base + 8])?;

            mux_inputs[..8].copy_from_slice(&addressed[base..base + 8]);
            mux_inputs[8..16].copy_from_slice(&inputs[0..8]);
            mux_inputs[16] = saves[half];
            self.data_muxes[half].conduct_into(&mux_inputs, &mut next[base..base + 8])?;

            // Store on the rising edge; the external pins gate the
            // outputs afterwards.
            let mut register_inputs = [Signal::HighImpedance; 11];
            register_inputs[..8].copy_from_slice(&next[base..base + 8]);
            register_inputs[8] = clock;
            register_inputs[9] = clock;
            register_inputs[10] = Signal::Driven(Bit::High);

            let mut unused = [Signal::HighImpedance; 8];
            register.conduct_into(&register_inputs, &mut unused)?;
        }

        self.gate_address(enable, outputs)?;
        self.gate_data(out_lo, out_hi, outputs);

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.registers
            .iter()
            .map(|register| register.transistor_count())
            .sum::<usize>()
            + self
                .half_adders
                .iter()
                .map(|adder| adder.transistor_count())
                .sum::<usize>()
            + self
                .count_muxes
                .iter()
                .map(|mux| mux.transistor_count())
                .sum::<usize>()
            + self
                .addr_muxes
                .iter()
                .map(|mux| mux.transistor_count())
                .sum::<usize>()
            + self
                .data_muxes
                .iter()
                .map(|mux| mux.transistor_count())
                .sum::<usize>()
            + self
                .address_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
            + self
                .data_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
    }
}

/// 16-bit stack pointer: an up/down counter with parallel load.
///
/// The same 16 full adders serve both directions. The per-bit adder
/// input `direction` is Low for a pop (adds `0000...0` with carry-in
/// High, i.e. increments by one) and High for a push (adds
/// `1111...1` with carry-in Low, i.e. decrements by one).
#[derive(Debug, Default)]
pub struct StackPointer16Bits {
    registers: [Register8Bits; 2],
    /// Full-adder chain computing `state + direction_pattern + carry_in`.
    full_adders: [FullAdder; 16],
    /// Inverts the direction line into the initial carry.
    not_direction: NotGate,
    /// OR of count_up and count_down: the counter is counting.
    counting_or: OrGate,
    /// `count_muxes[half]`: state when idle, computed next value when counting.
    count_muxes: [Mux8bits2x1; 2],
    /// `load_muxes[half]`: counter value or bus data, per half.
    load_muxes: [Mux8bits2x1; 2],
    output_nmos: [Nmos; 16],
}

impl StackPointer16Bits {
    /// The stored 16-bit value, least significant bit first.
    pub fn state(&self) -> [Bit; 16] {
        let low = self.registers[0].state();
        let high = self.registers[1].state();

        let mut value = [Bit::Low; 16];
        value[..8].copy_from_slice(&low);
        value[8..].copy_from_slice(&high);
        value
    }

    /// Gate the stored 16 bits through the output NMOS row.
    fn gate_output(&self, load: Signal, outputs: &mut [Signal]) -> Result<(), HardwareError> {
        let stored = self.state();

        for (index, nmos) in self.output_nmos.iter().enumerate() {
            nmos.conduct_into(
                &[load, Signal::from(stored[index])],
                &mut outputs[index..index + 1],
            )?;
        }

        Ok(())
    }
}

impl Component for StackPointer16Bits {
    /// Inputs:
    ///
    /// - 0..8:  data, least significant bit first (the bus, for loads)
    /// - 8:     clock
    /// - 9:     count_up — increment on the rising edge when High (pop)
    /// - 10:    count_down — decrement on the rising edge when High (push)
    /// - 11:    save_low — capture `data` into the low byte
    /// - 12:    save_high — capture `data` into the high byte
    /// - 13:    load — gate the 16-bit output onto the address bus
    ///
    /// Outputs:
    ///
    /// - 0..16: the counter value, gated by `load`.
    const INPUTS: usize = 14;
    const OUTPUTS: usize = 16;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let clock = inputs[8];
        let count_up = inputs[9];
        let count_down = inputs[10];
        let saves = [inputs[11], inputs[12]];
        let load = inputs[13];

        // With `FAST`, a High clock disables the registers' master
        // latches: the next value was preloaded during the Low tick
        // and the combinational next-value path is skipped entirely.
        if FAST && clock == Signal::Driven(Bit::High) {
            let mut register_inputs = [Signal::Driven(Bit::Low); 11];
            register_inputs[8] = clock;
            register_inputs[9] = clock;
            register_inputs[10] = Signal::Driven(Bit::High);

            let mut unused = [Signal::HighImpedance; 8];
            for register in &self.registers {
                register.conduct_into(&register_inputs, &mut unused)?;
            }

            return self.gate_output(load, outputs);
        }

        let mut counting = [Signal::HighImpedance; 1];
        self.counting_or
            .conduct_into(&[count_up, count_down], &mut counting)?;
        let count = counting[0];

        // direction Low -> add `0000...0` with carry-in High: +1 (pop).
        // direction High -> add `1111...1` with carry-in Low: -1 (push).
        let direction = count_down;

        let mut state = [Signal::HighImpedance; 16];
        for (index, bit) in self.state().iter().enumerate() {
            state[index] = Signal::from(*bit);
        }

        let mut next_value = [Signal::HighImpedance; 16];
        let mut not_direction = [Signal::HighImpedance; 1];
        self.not_direction
            .conduct_into(&[direction], &mut not_direction)?;
        let mut carry = not_direction[0];
        let mut fa_out = [Signal::HighImpedance; 2];

        for index in 0..16 {
            self.full_adders[index].conduct_into(&[state[index], direction, carry], &mut fa_out)?;
            next_value[index] = fa_out[0];
            carry = fa_out[1];
        }

        // Per half: mux1 selects hold/count on `count`, mux2 selects
        // counter value/bus data on the half's `save` line.
        let mut mid = [Signal::HighImpedance; 16];
        let mut next = [Signal::HighImpedance; 16];
        let mut mux_inputs = [Signal::HighImpedance; 17];

        for (half, register) in self.registers.iter().enumerate() {
            let base = half * 8;

            mux_inputs[..8].copy_from_slice(&state[base..base + 8]);
            mux_inputs[8..16].copy_from_slice(&next_value[base..base + 8]);
            mux_inputs[16] = count;
            self.count_muxes[half].conduct_into(&mux_inputs, &mut mid[base..base + 8])?;

            mux_inputs[..8].copy_from_slice(&mid[base..base + 8]);
            mux_inputs[8..16].copy_from_slice(&inputs[0..8]);
            mux_inputs[16] = saves[half];
            self.load_muxes[half].conduct_into(&mux_inputs, &mut next[base..base + 8])?;

            let mut register_inputs = [Signal::HighImpedance; 11];
            register_inputs[..8].copy_from_slice(&next[base..base + 8]);
            register_inputs[8] = clock;
            register_inputs[9] = clock;
            register_inputs[10] = Signal::Driven(Bit::High);

            let mut unused = [Signal::HighImpedance; 8];
            register.conduct_into(&register_inputs, &mut unused)?;
        }

        self.gate_output(load, outputs)
    }

    fn transistor_count(&self) -> usize {
        self.registers
            .iter()
            .map(|register| register.transistor_count())
            .sum::<usize>()
            + self
                .full_adders
                .iter()
                .map(|adder| adder.transistor_count())
                .sum::<usize>()
            + self.not_direction.transistor_count()
            + self.counting_or.transistor_count()
            + self
                .count_muxes
                .iter()
                .map(|mux| mux.transistor_count())
                .sum::<usize>()
            + self
                .load_muxes
                .iter()
                .map(|mux| mux.transistor_count())
                .sum::<usize>()
            + self
                .output_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
    }
}

/// Eight tri-state input switches: the value drives the data bus only
/// while `enable` is High (the IN instruction).
#[derive(Debug, Default)]
pub struct InputPort {
    output_nmos: [Nmos; 8],
}

impl Component for InputPort {
    /// Inputs:
    ///
    /// - 0..8: the port value (physical switches / external device)
    /// - 8:    enable
    ///
    /// Outputs:
    ///
    /// - 0..8: the port value, gated by `enable`.
    const INPUTS: usize = 9;
    const OUTPUTS: usize = 8;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        for (index, nmos) in self.output_nmos.iter().enumerate() {
            nmos.conduct_into(&[inputs[8], inputs[index]], &mut outputs[index..index + 1])?;
        }

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.output_nmos
            .iter()
            .map(|nmos| nmos.transistor_count())
            .sum::<usize>()
    }
}

/// The OUT latch: captures the accumulator value driven on the bus
/// and holds it for display outside the CPU. Its outputs are always
/// driven — they feed the outside world, not the shared bus.
#[derive(Debug, Default)]
pub struct OutputRegister {
    register: Register8Bits,
}

impl OutputRegister {
    /// The displayed byte, least significant bit first.
    pub fn state(&self) -> [Bit; 8] {
        self.register.state()
    }
}

impl Component for OutputRegister {
    /// Inputs:
    ///
    /// - 0..8: data, least significant bit first
    /// - 8:    clock
    /// - 9:    save (the OUT instruction's write pulse)
    ///
    /// Outputs:
    ///
    /// - 0..8: the displayed byte, always driven.
    const INPUTS: usize = 10;
    const OUTPUTS: usize = 8;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let mut register_inputs = [Signal::HighImpedance; 11];
        register_inputs[..8].copy_from_slice(&inputs[0..8]);
        register_inputs[8] = inputs[8];
        register_inputs[9] = inputs[9];
        // The latch drives the outside world directly: `load` tied HIGH.
        register_inputs[10] = Signal::Driven(Bit::High);

        let mut stored = [Signal::HighImpedance; 8];
        self.register.conduct_into(&register_inputs, &mut stored)?;

        outputs.copy_from_slice(&stored);

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.register.transistor_count()
    }
}

#[cfg(test)]
fn signals(bits: &[Bit]) -> Vec<Signal> {
    bits.iter().copied().map(Signal::from).collect()
}

#[cfg(test)]
mod register8bits_tests {
    use super::*;
    use Bit::{High, Low};

    const DATA: [Bit; 8] = [High, Low, High, Low, High, Low, High, Low];

    fn register_inputs(data: [Bit; 8], clock: Bit, save: Bit, load: Bit) -> Vec<Bit> {
        data.into_iter().chain([clock, save, load]).collect()
    }

    #[test]
    fn test_write_and_read_sequence() {
        let register = Register8Bits::default();

        assert_eq!(
            register.compute(&register_inputs([Low; 8], Low, Low, High)),
            Ok(vec![Low; 8])
        );

        // clock LOW: nothing written
        assert_eq!(
            register.compute(&register_inputs(DATA, Low, Low, High)),
            Ok(vec![Low; 8])
        );
        assert_eq!(
            register.compute(&register_inputs(DATA, High, Low, High)),
            Ok(vec![Low; 8])
        );
        assert_eq!(
            register.compute(&register_inputs(DATA, Low, Low, High)),
            Ok(vec![Low; 8])
        );

        // save LOW: nothing written even on a rising edge
        assert_eq!(
            register.compute(&register_inputs(DATA, High, Low, High)),
            Ok(vec![Low; 8])
        );
        assert_eq!(
            register.compute(&register_inputs(DATA, Low, Low, High)),
            Ok(vec![Low; 8])
        );

        // rising edge with save HIGH
        assert_eq!(
            register.compute(&register_inputs(DATA, High, High, High)),
            Ok(DATA.to_vec())
        );

        // the value is kept
        assert_eq!(
            register.compute(&register_inputs(DATA, Low, Low, High)),
            Ok(DATA.to_vec())
        );
        assert_eq!(
            register.compute(&register_inputs(DATA, Low, High, High)),
            Ok(DATA.to_vec())
        );

        // load LOW floats the outputs
        assert_eq!(
            register.conduct(&signals(&register_inputs(DATA, Low, High, Low))),
            Ok(vec![Signal::HighImpedance; 8])
        );
    }

    #[test]
    fn test_state() {
        let register = Register8Bits::default();
        assert_eq!(register.state(), [Low; 8]);

        let data = [High, High, Low, Low, High, Low, High, Low];

        register
            .compute(&register_inputs(data, Low, High, High))
            .expect("computation failed");
        register
            .compute(&register_inputs(data, High, High, High))
            .expect("computation failed");

        assert_eq!(register.state(), data);
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(Register8Bits::default().transistor_count(), 544);
    }
}

#[cfg(test)]
mod programcounter4bits_tests {
    use super::*;
    use Bit::{High, Low};

    fn pc_inputs(data: [Bit; 4], clock: Bit, save: Bit, load: Bit) -> Vec<Bit> {
        data.into_iter().chain([clock, save, load]).collect()
    }

    #[test]
    fn test_count_load_and_wrap_sequence() {
        let counter = ProgramCounter4Bits::default();

        assert_eq!(
            counter.compute(&pc_inputs([Low; 4], Low, Low, High)),
            Ok(vec![Low; 4])
        );

        // increment
        assert_eq!(
            counter.compute(&pc_inputs([Low; 4], High, Low, High)),
            Ok(vec![High, Low, Low, Low])
        );

        // no edge
        assert_eq!(
            counter.compute(&pc_inputs([Low; 4], Low, Low, High)),
            Ok(vec![High, Low, Low, Low])
        );

        assert_eq!(
            counter.compute(&pc_inputs([High; 4], High, High, High)),
            Ok(vec![Low, High, Low, Low])
        );
        assert_eq!(
            counter.compute(&pc_inputs([High; 4], Low, High, High)),
            Ok(vec![Low, High, Low, Low])
        );

        // load 15 (captured by the master latch, then by the slave on
        // the edge)
        assert_eq!(
            counter.compute(&pc_inputs([Low; 4], High, High, High)),
            Ok(vec![High; 4])
        );
        assert_eq!(
            counter.compute(&pc_inputs([Low; 4], Low, Low, High)),
            Ok(vec![High; 4])
        );

        // 15 + 1 wraps
        assert_eq!(
            counter.compute(&pc_inputs([Low; 4], High, Low, High)),
            Ok(vec![Low; 4])
        );
        assert_eq!(
            counter.compute(&pc_inputs([Low; 4], Low, Low, High)),
            Ok(vec![Low; 4])
        );
        assert_eq!(
            counter.compute(&pc_inputs([Low; 4], High, Low, High)),
            Ok(vec![High, Low, Low, Low])
        );
    }

    #[test]
    fn test_load_low_floats_the_outputs() {
        let counter = ProgramCounter4Bits::default();

        assert_eq!(
            counter.conduct(&signals(&pc_inputs([Low, Low, Low, High], Low, Low, Low))),
            Ok(vec![Signal::HighImpedance; 4])
        );
        assert_eq!(
            counter.conduct(&signals(&pc_inputs([Low, Low, Low, High], High, Low, Low))),
            Ok(vec![Signal::HighImpedance; 4])
        );
    }

    #[test]
    fn test_transistor_count() {
        // 544 + 400 + 2 x 160 + 4 = 1268. The Rust XorGate is built
        // from four NAND gates, exactly like the Python one.
        assert_eq!(ProgramCounter4Bits::default().transistor_count(), 1268);
    }
}

#[cfg(test)]
mod ram256bits_tests {
    use super::*;
    use Bit::{High, Low};

    const DATA_A: [Bit; 8] = [High, Low, High, Low, High, Low, High, Low];
    const DATA_B: [Bit; 8] = [Low, High, Low, High, Low, High, Low, High];

    fn ram_inputs(data: [Bit; 8], address: [Bit; 4], clock: Bit, save: Bit, load: Bit) -> Vec<Bit> {
        data.into_iter()
            .chain(address)
            .chain([clock, save, load])
            .collect()
    }

    #[test]
    fn test_write_and_read_back() {
        let ram = Ram256Bits::default();

        // Write at address 0 with a LOW->HIGH clock sequence.
        ram.conduct(&signals(&ram_inputs(DATA_A, [Low; 4], Low, High, Low)))
            .expect("write failed");
        ram.conduct(&signals(&ram_inputs(DATA_A, [Low; 4], High, High, Low)))
            .expect("write failed");
        assert_eq!(
            ram.compute(&ram_inputs([Low; 8], [Low; 4], Low, Low, High)),
            Ok(DATA_A.to_vec())
        );

        // Write at address 3: address bits LSB-first (1, 1, 0, 0).
        ram.conduct(&signals(&ram_inputs(
            DATA_B,
            [High, High, Low, Low],
            Low,
            High,
            Low,
        )))
        .expect("write failed");
        ram.conduct(&signals(&ram_inputs(
            DATA_B,
            [High, High, Low, Low],
            High,
            High,
            Low,
        )))
        .expect("write failed");
        assert_eq!(
            ram.compute(&ram_inputs(
                [Low; 8],
                [High, High, Low, Low],
                Low,
                Low,
                High
            )),
            Ok(DATA_B.to_vec())
        );

        // The first cell is unchanged.
        assert_eq!(
            ram.compute(&ram_inputs([Low; 8], [Low; 4], Low, Low, High)),
            Ok(DATA_A.to_vec())
        );

        // Never-written cells read as zero.
        assert_eq!(
            ram.compute(&ram_inputs([Low; 8], [Low, Low, High, Low], Low, Low, High)),
            Ok(vec![Low; 8])
        );
    }

    #[test]
    fn test_state() {
        let ram = Ram256Bits::default();

        ram.conduct(&signals(&ram_inputs(DATA_A, [Low; 4], Low, High, Low)))
            .expect("write failed");
        ram.conduct(&signals(&ram_inputs(DATA_A, [Low; 4], High, High, Low)))
            .expect("write failed");

        assert_eq!(ram.state()[0], DATA_A);
        assert_eq!(ram.state()[1], [Low; 8]);
    }

    #[test]
    fn test_load_low_floats_the_bus() {
        let outputs = Ram256Bits::default()
            .conduct(&signals(&ram_inputs([High; 8], [Low; 4], Low, Low, Low)))
            .expect("computation failed");

        assert_eq!(outputs, vec![Signal::HighImpedance; 8]);
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(Ram256Bits::default().transistor_count(), 9048);
    }
}

#[cfg(test)]
mod ram64kbits_tests {
    use super::*;
    use Bit::{High, Low};

    /// Address, LSB first.
    fn address_bits(address: usize) -> Vec<Bit> {
        (0..16)
            .map(|index| {
                if (address >> index) & 1 == 1 {
                    High
                } else {
                    Low
                }
            })
            .collect()
    }

    fn ram_inputs(data: [Bit; 8], address: usize, clock: Bit, save: Bit, load: Bit) -> Vec<Bit> {
        data.into_iter()
            .chain(address_bits(address))
            .chain([clock, save, load])
            .collect()
    }

    /// A set of addresses covering the corners of the space: first and
    /// last registers of the first and last chips, both sides of a
    /// chip boundary, both sides of a bank boundary, and the very last
    /// byte.
    const SAMPLED_ADDRESSES: [usize; 8] = [
        0, 15, 16, 271,  // 0x10F: last register of chip 16
        4095, // 0xFFF: last byte of bank 0
        4096, // 0x1000: first byte of bank 1
        0xAA55, 65535,
    ];

    #[test]
    fn test_write_byte_and_read_back() {
        let ram = Ram64KBits::default();

        for (index, &address) in SAMPLED_ADDRESSES.iter().enumerate() {
            let data = [(index as u8) | 0x10; 8].map(|byte| if byte & 1 == 1 { High } else { Low });
            ram.write_byte(address, data).expect("write failed");
            assert_eq!(ram.state(address), data, "read back failed at {address}");
        }

        // An untouched address still reads zero.
        assert_eq!(ram.state(1234), [Low; 8]);
    }

    #[test]
    fn test_conduct_write_and_read_back() {
        let ram = Ram64KBits::default();
        let data = [High, Low, High, High, Low, Low, High, Low];
        let address = 0x1234; // bank 0x12, chip 3, register 4

        // Write with a LOW->HIGH->LOW clock sequence.
        ram.conduct(&signals(&ram_inputs(data, address, Low, High, Low)))
            .expect("write failed");
        ram.conduct(&signals(&ram_inputs(data, address, High, High, Low)))
            .expect("write failed");
        ram.conduct(&signals(&ram_inputs(data, address, Low, Low, Low)))
            .expect("write failed");

        // Read back through the bus.
        let read = ram
            .compute(&ram_inputs([Low; 8], address, Low, Low, High))
            .expect("read failed");
        assert_eq!(read, data.to_vec());
        assert_eq!(ram.state(address), data);
    }

    #[test]
    fn test_read_only_selected_chip_drives() {
        let ram = Ram64KBits::default();
        let data = [High; 8];

        ram.write_byte(0x0100, data).expect("write failed");

        // Reading a neighbouring address yields zero, not the stored
        // byte: only the selected chip drives the bus.
        let read = ram
            .compute(&ram_inputs([Low; 8], 0x0101, Low, Low, High))
            .expect("read failed");
        assert_eq!(read, vec![Low; 8]);

        let read = ram
            .compute(&ram_inputs([Low; 8], 0x0100, Low, Low, High))
            .expect("read failed");
        assert_eq!(read, data.to_vec());
    }

    #[test]
    fn test_load_low_floats_the_bus() {
        let ram = Ram64KBits::default();
        ram.write_byte(42, [High; 8]).expect("write failed");

        let outputs = ram
            .conduct(&signals(&ram_inputs([High; 8], 42, Low, Low, Low)))
            .expect("computation failed");

        assert_eq!(outputs, vec![Signal::HighImpedance; 8]);
    }

    #[test]
    fn test_rejects_wrong_input_count() {
        let ram = Ram64KBits::default();

        assert_eq!(
            ram.conduct(&[Signal::HighImpedance; 26]),
            Err(HardwareError::InvalidInputCount {
                expected: 27,
                actual: 26,
            })
        );
    }

    #[test]
    #[should_panic(expected = "address must be within")]
    fn test_write_byte_rejects_out_of_range_address() {
        let ram = Ram64KBits::default();
        let _ = ram.write_byte(65536, [Low; 8]);
    }

    #[test]
    fn test_transistor_count() {
        // 4096 Ram256Bits chips + 3 decoders + one AND per bank line
        // and three ANDs per chip (select, save, load).
        let expected = RAM64K_CHIPS * Ram256Bits::default().transistor_count()
            + 3 * Decoder4to16::default().transistor_count()
            + 6 * (RAM64K_BANKS + 3 * RAM64K_CHIPS);
        assert_eq!(Ram64KBits::default().transistor_count(), expected);
    }
}

#[cfg(test)]
mod sap2_address_path_tests {
    use super::*;
    use Bit::{High, Low};

    fn pulse(conduct_fn: impl Fn(Bit) -> Result<Vec<Signal>, HardwareError>) {
        conduct_fn(Low).expect("low tick failed");
        conduct_fn(High).expect("high tick failed");
        conduct_fn(Low).expect("settle tick failed");
    }

    fn to_int(bits: &[Bit]) -> usize {
        bits.iter().enumerate().fold(0, |acc, (index, bit)| {
            acc | (u32::from(*bit == High) << index) as usize
        })
    }

    #[test]
    fn test_flags_register_capture_and_hold() {
        let flags = FlagsRegister::default();
        assert_eq!(flags.state(), (Low, Low, Low));

        // DFlipFlopSaveLoad has two outputs (q, q_bar); the flags only
        // expose q, so each pulse routes through a 2-slot buffer.
        let flag_pulse = |z: Bit, c: Bit, s: Bit, save: Bit| {
            let run = |clock: Bit| {
                let mut out = [Signal::HighImpedance; 3];
                flags.conduct_into(
                    &[
                        Signal::from(z),
                        Signal::from(c),
                        Signal::from(s),
                        Signal::from(clock),
                        Signal::from(save),
                    ],
                    &mut out,
                )
            };
            run(Low).expect("low tick failed");
            run(High).expect("high tick failed");
            run(Low).expect("settle tick failed");
        };

        // Capture (zero=High, carry=Low, sign=High) on a clock pulse.
        flag_pulse(High, Low, High, High);
        assert_eq!(flags.state(), (High, Low, High));

        // With save Low, the flags hold their value.
        flag_pulse(Low, High, Low, Low);
        assert_eq!(flags.state(), (High, Low, High));

        // A new save pulse captures the new inputs.
        flag_pulse(Low, Low, Low, High);
        assert_eq!(flags.state(), (Low, Low, Low));
    }

    #[test]
    fn test_memory_address_register_two_halves() {
        let mar = MemoryAddressRegister16Bits::default();
        assert_eq!(mar.state(), [Low; 16]);

        // Helper: `data` on the data bus, zeros on the address bus,
        // per-half saves plus the 16-bit load.
        let run = |clock: Bit, data: u8, save_low: Bit, save_high: Bit, load16: Bit| {
            let data_bits: Vec<Bit> = (0..8)
                .map(|i| if (data >> i) & 1 == 1 { High } else { Low })
                .collect();
            mar.conduct(&signals(
                &data_bits
                    .into_iter()
                    .chain([Low; 16])
                    .chain([clock, save_low, save_high, load16, High])
                    .collect::<Vec<Bit>>(),
            ))
        };

        // Load the low byte 0x34 = bits 2, 4, 5...
        pulse(|clock| run(clock, 0x34, High, Low, Low));
        assert_eq!(to_int(&mar.state()) & 0xFF, 0x34);

        // ...then the high byte 0x12 = bits 1, 4.
        pulse(|clock| run(clock, 0x12, Low, High, Low));
        assert_eq!(to_int(&mar.state()), 0x1234);

        // 16-bit load from the address bus: 0xABCD.
        let addr: Vec<Bit> = (0..16)
            .map(|i| if (0xABCD >> i) & 1 == 1 { High } else { Low })
            .collect();
        let run16 = |clock: Bit| {
            mar.conduct(&signals(
                &[Low; 8]
                    .into_iter()
                    .chain(addr.iter().copied())
                    .chain([clock, Low, Low, High, High])
                    .collect::<Vec<Bit>>(),
            ))
        };
        pulse(run16);
        assert_eq!(to_int(&mar.state()), 0xABCD);

        // The outputs drive the stored address when enabled.
        let outputs = mar
            .conduct(&signals(
                &[Low; 24]
                    .into_iter()
                    .chain([Low, Low, Low, Low, High])
                    .collect::<Vec<Bit>>(),
            ))
            .expect("conduct failed");
        let expected: Vec<Bit> = (0..16)
            .map(|i| if (0xABCD >> i) & 1 == 1 { High } else { Low })
            .collect();
        assert_eq!(
            outputs
                .iter()
                .map(|s| match s {
                    Signal::Driven(bit) => *bit,
                    Signal::HighImpedance => panic!("floating address output"),
                })
                .collect::<Vec<Bit>>(),
            expected
        );
    }
}

#[cfg(test)]
mod program_counter_16_tests {
    use super::*;
    use Bit::{High, Low};

    fn pulse(pc: &ProgramCounter16Bits, data: u8, count: Bit, save_low: Bit, save_high: Bit) {
        let run = |clock: Bit| {
            let data_bits: Vec<Bit> = (0..8)
                .map(|i| if (data >> i) & 1 == 1 { High } else { Low })
                .collect();
            pc.conduct(&signals(
                &data_bits
                    .into_iter()
                    .chain([Low; 16]) // address bus input
                    .chain([clock, count, Low, save_low, save_high, High, Low, Low])
                    .collect::<Vec<Bit>>(),
            ))
        };

        run(Low).expect("low tick failed");
        run(High).expect("high tick failed");
        run(Low).expect("settle tick failed");
    }

    fn to_int(bits: &[Bit]) -> usize {
        bits.iter().enumerate().fold(0, |acc, (index, bit)| {
            acc | (u32::from(*bit == High) << index) as usize
        })
    }

    #[test]
    fn test_counts_from_zero() {
        let pc = ProgramCounter16Bits::default();

        for expected in 1..=5 {
            pulse(&pc, 0, High, Low, Low);
            assert_eq!(to_int(&pc.state()), expected);
        }
    }

    #[test]
    fn test_loads_two_halves_then_counts_across_byte_boundary() {
        let pc = ProgramCounter16Bits::default();

        pulse(&pc, 0x34, Low, High, Low); // low half
        pulse(&pc, 0x12, Low, Low, High); // high half
        assert_eq!(to_int(&pc.state()), 0x1234);

        // Count until the carry crosses the byte boundary.
        for _ in 0..(0xFF - 0x34) {
            pulse(&pc, 0, High, Low, Low);
        }
        assert_eq!(to_int(&pc.state()), 0x12FF);

        pulse(&pc, 0, High, Low, Low);
        assert_eq!(to_int(&pc.state()), 0x1300);
    }

    #[test]
    fn test_holds_when_count_is_low() {
        let pc = ProgramCounter16Bits::default();

        pulse(&pc, 42, Low, High, Low);
        pulse(&pc, 0, Low, Low, Low);
        assert_eq!(to_int(&pc.state()), 42);
    }

    #[test]
    fn test_load_gates_the_outputs() {
        let pc = ProgramCounter16Bits::default();
        pulse(&pc, 7, Low, High, Low);

        // enable Low: the address outputs float.
        let outputs = pc.conduct(&signals(&[Low; 32])).expect("conduct failed");
        assert_eq!(outputs[0..16], [Signal::HighImpedance; 16]);
        assert_eq!(outputs[16..24], [Signal::HighImpedance; 8]);

        // enable High: the address outputs drive the stored value.
        let outputs = pc
            .conduct(&signals(
                &[Low; 8]
                    .into_iter()
                    .chain([Low; 16])
                    .chain([Low, Low, Low, Low, Low, High, Low, Low])
                    .collect::<Vec<Bit>>(),
            ))
            .expect("conduct failed");
        assert_eq!(
            to_int(
                &outputs[0..16]
                    .iter()
                    .map(|s| match s {
                        Signal::Driven(bit) => *bit,
                        Signal::HighImpedance => panic!("floating address output"),
                    })
                    .collect::<Vec<Bit>>()
            ),
            7
        );
        assert_eq!(outputs[16..24], [Signal::HighImpedance; 8]);
    }

    #[test]
    fn test_load_addr_captures_the_address_bus() {
        let pc = ProgramCounter16Bits::default();

        // Jump: capture 0x1234 from the address bus in one pulse.
        let addr: Vec<Bit> = (0..16)
            .map(|i| if (0x1234 >> i) & 1 == 1 { High } else { Low })
            .collect();
        let run = |clock: Bit| {
            pc.conduct(&signals(
                &[Low; 8]
                    .into_iter()
                    .chain(addr.iter().copied())
                    .chain([clock, Low, High, Low, Low, High, Low, Low])
                    .collect::<Vec<Bit>>(),
            ))
        };
        run(Low).expect("low tick failed");
        run(High).expect("high tick failed");
        run(Low).expect("settle tick failed");
        assert_eq!(to_int(&pc.state()), 0x1234);

        // save_low still wins over load_addr (data-bus priority).
        pulse(&pc, 0x99, Low, High, Low);
        assert_eq!(to_int(&pc.state()), 0x1299);
    }

    #[test]
    fn test_out_lo_and_out_hi_drive_the_data_byte() {
        let pc = ProgramCounter16Bits::default();
        pulse(&pc, 0x34, Low, High, Low); // low = 0x34
        pulse(&pc, 0x12, Low, Low, High); // high = 0x12

        let byte_of = |out_lo: Bit, out_hi: Bit| -> Option<usize> {
            let outputs = pc
                .conduct(&signals(
                    &[Low; 8]
                        .into_iter()
                        .chain([Low; 16])
                        .chain([Low, Low, Low, Low, Low, Low, out_lo, out_hi])
                        .collect::<Vec<Bit>>(),
                ))
                .expect("conduct failed");
            if outputs[16..24].iter().all(|s| *s == Signal::HighImpedance) {
                None
            } else {
                Some(to_int(
                    &outputs[16..24]
                        .iter()
                        .map(|s| match s {
                            Signal::Driven(bit) => *bit,
                            Signal::HighImpedance => panic!("floating data byte"),
                        })
                        .collect::<Vec<Bit>>(),
                ))
            }
        };

        assert_eq!(byte_of(High, Low), Some(0x34));
        assert_eq!(byte_of(Low, High), Some(0x12));
        assert_eq!(byte_of(Low, Low), None);
    }
}

#[cfg(test)]
mod sap2_stack_and_io_tests {
    use super::*;
    use Bit::{High, Low};

    fn to_int(bits: &[Bit]) -> usize {
        bits.iter().enumerate().fold(0, |acc, (index, bit)| {
            acc | (u32::from(*bit == High) << index) as usize
        })
    }

    fn pulse(
        sp: &StackPointer16Bits,
        data: u8,
        count_up: Bit,
        count_down: Bit,
        save_low: Bit,
        save_high: Bit,
    ) {
        let run = |clock: Bit| {
            let data_bits: Vec<Bit> = (0..8)
                .map(|i| if (data >> i) & 1 == 1 { High } else { Low })
                .collect();
            sp.conduct(&signals(
                &data_bits
                    .into_iter()
                    .chain([clock, count_up, count_down, save_low, save_high, Low])
                    .collect::<Vec<Bit>>(),
            ))
        };

        run(Low).expect("low tick failed");
        run(High).expect("high tick failed");
        run(Low).expect("settle tick failed");
    }

    #[test]
    fn test_stack_pointer_increments_and_decrements() {
        let sp = StackPointer16Bits::default();

        // Initialize to 0x2000 (top of memory area).
        pulse(&sp, 0x00, Low, Low, High, Low);
        pulse(&sp, 0x20, Low, Low, Low, High);
        assert_eq!(to_int(&sp.state()), 0x2000);

        // Push: decrement twice, crossing a byte boundary.
        pulse(&sp, 0, Low, High, Low, Low);
        pulse(&sp, 0, Low, High, Low, Low);
        assert_eq!(to_int(&sp.state()), 0x1FFE);

        // Pop: increment back twice.
        pulse(&sp, 0, High, Low, Low, Low);
        pulse(&sp, 0, High, Low, Low, Low);
        assert_eq!(to_int(&sp.state()), 0x2000);
    }

    #[test]
    fn test_stack_pointer_holds_when_idle() {
        let sp = StackPointer16Bits::default();

        pulse(&sp, 0xAB, Low, Low, High, Low);
        pulse(&sp, 0, Low, Low, Low, Low);
        assert_eq!(to_int(&sp.state()), 0xAB);
    }

    #[test]
    fn test_input_port_drives_only_when_enabled() {
        let port = InputPort::default();
        let value: Vec<Bit> = [High, Low, High, Low, High, Low, High, Low].to_vec();

        // Enabled: the value drives the bus.
        let inputs: Vec<Bit> = value.clone().into_iter().chain([High]).collect();
        let outputs = port.conduct(&signals(&inputs)).expect("conduct failed");
        let bits: Vec<Bit> = outputs
            .iter()
            .map(|s| match s {
                Signal::Driven(bit) => *bit,
                Signal::HighImpedance => panic!("floating output"),
            })
            .collect();
        assert_eq!(bits, value);

        // Disabled: everything floats.
        let inputs: Vec<Bit> = value.into_iter().chain([Low]).collect();
        let outputs = port.conduct(&signals(&inputs)).expect("conduct failed");
        assert_eq!(outputs, vec![Signal::HighImpedance; 8]);
    }

    #[test]
    fn test_output_register_latches_the_value() {
        let out = OutputRegister::default();
        assert_eq!(out.state(), [Low; 8]);

        // Latch 0x5A = bits 1, 3, 4, 6 on a clock pulse.
        let run = |clock: Bit, save: Bit| {
            out.conduct(&signals(&[
                Low, High, Low, High, High, Low, High, Low, clock, save,
            ]))
        };
        run(Low, High).expect("low tick failed");
        run(High, High).expect("high tick failed");
        run(Low, Low).expect("settle tick failed");

        assert_eq!(to_int(&out.state()), 0x5A);

        // Without a save pulse, the displayed value holds.
        run(Low, Low).expect("idle tick failed");
        run(High, Low).expect("idle tick failed");
        assert_eq!(to_int(&out.state()), 0x5A);
    }
}
