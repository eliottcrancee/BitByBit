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

use crate::FAST_MODE;
use crate::arithmetic::Adder8Bits;
use crate::decoder::Decoder4to16;
use crate::gates::AndGate;
use crate::hardware::{Bit, Component, HardwareError, Nmos, Signal, bus8};
use crate::latches::DFlipFlopSaveLoad;
use crate::mux::Mux8bits2x1;

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
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        if inputs.len() != 11 {
            return Err(HardwareError::InvalidInputCount {
                expected: 11,
                actual: inputs.len(),
            });
        }

        let clock = inputs[8];
        let save = inputs[9];
        let load = inputs[10];

        let mut outputs = Vec::with_capacity(8);

        for (index, flip_flop) in self.flip_flops.iter().enumerate() {
            let output = flip_flop.conduct(&[inputs[index], clock, save, load])?;
            outputs.push(output[0]);
        }

        Ok(outputs)
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
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        if inputs.len() != 7 {
            return Err(HardwareError::InvalidInputCount {
                expected: 7,
                actual: inputs.len(),
            });
        }

        let clock = inputs[4];
        let save = inputs[5];
        let load = inputs[6];

        // incremented = state + 1, LSB first. The adder also returns a
        // carry bit, which is not needed here.
        let state = self.register.state();
        let mut adder_inputs: Vec<Signal> = Vec::with_capacity(17);
        adder_inputs.extend(state.iter().copied().map(Signal::from));
        adder_inputs.extend_from_slice(&ONE_BYTE);
        adder_inputs.push(Signal::Driven(Bit::Low));
        let incremented = self.adder.conduct(&adder_inputs)?;

        // selected = save ? data : state + 1
        //
        // `data` must be zero-extended to 8 bits for the multiplexer.
        let mut load_inputs: Vec<Signal> = Vec::with_capacity(17);
        load_inputs.extend_from_slice(&incremented[..8]);
        load_inputs.extend_from_slice(&inputs[0..4]);
        load_inputs.extend_from_slice(&ZERO_BYTE[..4]);
        load_inputs.push(save);
        let selected = self.load_mux.conduct(&load_inputs)?;

        // wrapped = selected[4] ? 0 : selected
        //
        // A 4-bit counter only exposes its low bits; counting past 15
        // sets the fifth bit, which resets the stored value to zero.
        let mut overflow_inputs: Vec<Signal> = Vec::with_capacity(17);
        overflow_inputs.extend_from_slice(&selected);
        overflow_inputs.extend_from_slice(&ZERO_BYTE);
        overflow_inputs.push(selected[4]);
        let wrapped = self.overflow_mux.conduct(&overflow_inputs)?;

        // Store `wrapped` on the rising edge; the register outputs are
        // always driven, the external `load` gates them afterwards.
        let mut register_inputs: Vec<Signal> = Vec::with_capacity(11);
        register_inputs.extend_from_slice(&wrapped);
        register_inputs.extend_from_slice(&[clock, clock, Signal::Driven(Bit::High)]);
        let stored = self.register.conduct(&register_inputs)?;

        let mut outputs = Vec::with_capacity(ADDRESS_BITS);

        for (nmos, stored_bit) in self.output_nmos.iter().zip(stored.iter()) {
            let output = nmos.conduct(&[load, *stored_bit])?;
            outputs.push(output[0]);
        }

        Ok(outputs)
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

        let inputs_for = |clock: Bit, save: Bit| -> Vec<Signal> {
            data.iter()
                .copied()
                .map(Signal::from)
                .chain([
                    Signal::from(clock),
                    Signal::from(save),
                    Signal::Driven(Bit::Low),
                ])
                .collect()
        };

        // Master latch receives the data while the clock is LOW, the
        // slave latch updates during HIGH, then the clock returns LOW.
        register.conduct(&inputs_for(Bit::Low, Bit::High))?;
        register.conduct(&inputs_for(Bit::High, Bit::High))?;
        register.conduct(&inputs_for(Bit::Low, Bit::Low))?;

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
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        if inputs.len() != 15 {
            return Err(HardwareError::InvalidInputCount {
                expected: 15,
                actual: inputs.len(),
            });
        }

        let data = &inputs[0..8];
        let address = &inputs[8..12];
        let clock = inputs[12];
        let save = inputs[13];
        let load = inputs[14];

        let selected = self.decoder.conduct(address)?;

        let mut drivers: Vec<[Signal; 8]> = Vec::with_capacity(RAM_REGISTERS);

        for (index, select_line) in selected.iter().enumerate() {
            let register_save = self.save_gates[index].conduct(&[*select_line, save])?[0];
            let register_load = self.load_gates[index].conduct(&[*select_line, load])?[0];

            // With `FAST_MODE`, an unselected register receives neither
            // the save nor the load signal: it cannot change state nor
            // drive the bus, so its flip-flops are left untouched (their
            // outputs stay floating on the bus row).
            if FAST_MODE
                && register_save == Signal::Driven(Bit::Low)
                && register_load == Signal::Driven(Bit::Low)
            {
                drivers.push([Signal::HighImpedance; 8]);
                continue;
            }

            let mut register_inputs: Vec<Signal> = Vec::with_capacity(11);
            register_inputs.extend_from_slice(data);
            register_inputs.extend_from_slice(&[clock, register_save, register_load]);

            let register_output = self.registers[index].conduct(&register_inputs)?;

            drivers.push(
                register_output
                    .try_into()
                    .expect("register always outputs 8 signals"),
            );
        }

        Ok(bus8(drivers)?.to_vec())
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
