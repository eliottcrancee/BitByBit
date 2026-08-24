//! Sequential circuits built on top of the gates from [`crate::gates`]
//! and the multiplexer from [`crate::mux`].
//!
//! These components hold state. The trait [`crate::hardware::Component`]
//! takes `&self`, so the storage cells use interior mutability
//! (`Cell<Bit>`): calling `conduct` both propagates the inputs and
//! updates the stored value, mirroring the Python `__call__` semantics.
//!
//! Components, from the simplest to the most elaborate:
//!
//!     SRLatch
//!         Two cross-coupled NOR gates. Stores one bit. SET and RESET
//!         both HIGH is forbidden.
//!
//!     DLatch
//!         An SR latch fed by `data` and `NOT data`, gated by
//!         `enable`. Transparent while `enable` is HIGH.
//!
//!     DFlipFlop
//!         Master-slave pair of D latches. Updates `q` only on the
//!         rising edge of `clock`.
//!
//!     DFlipFlopSave
//!         A D flip-flop whose write is gated by `save`: on a rising
//!         edge, `q` keeps its previous value when `save` is LOW.
//!
//!     DFlipFlopSaveLoad
//!         A D flip-flop with save, whose outputs are gated by `load`.
//!         When `load` is LOW the outputs float in high impedance.
//!
//!     OneHotCounter6Bits
//!         A 6-bit ring of D flip-flops. Exactly one bit is HIGH and it
//!         advances by one position on each rising clock edge.

use std::cell::Cell;

use crate::gates::{AndGate, NorGate, NotGate};
use crate::hardware::{Bit, Component, HardwareError, Nmos, Signal, stabilize};
use crate::mux::Mux2x1;

pub const COUNTER_BITS: usize = 6;
const MAX_STABILIZATION_ITERATIONS: usize = 3;

/// SR latch made of two cross-coupled NOR gates.
#[derive(Debug)]
pub struct SRLatch {
    nor1: NorGate,
    nor2: NorGate,
    q: Cell<Bit>,
    q_bar: Cell<Bit>,
}

impl Default for SRLatch {
    fn default() -> Self {
        Self {
            nor1: NorGate::default(),
            nor2: NorGate::default(),
            q: Cell::new(Bit::Low),
            q_bar: Cell::new(Bit::High),
        }
    }
}

impl SRLatch {
    /// The stored output.
    pub fn q(&self) -> Bit {
        self.q.get()
    }

    /// The complement of the stored output.
    pub fn q_bar(&self) -> Bit {
        self.q_bar.get()
    }
}

impl Component for SRLatch {
    /// Inputs:
    ///
    /// - 0: set
    /// - 1: reset
    ///
    /// Outputs:
    ///
    /// - 0: q
    /// - 1: q_bar
    ///
    /// SET High / RESET Low sets the latch, SET Low / RESET High resets
    /// it and SET Low / RESET Low keeps the previous state. Both inputs
    /// HIGH is rejected: Q and Q_bar would stop being complementary,
    /// and real silicon becomes metastable when both inputs are
    /// released together.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [set_input, reset_input] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let set = Signal::from(Bit::try_from(*set_input)?);
        let reset = Signal::from(Bit::try_from(*reset_input)?);

        if set == Signal::from(Bit::High) && reset == Signal::from(Bit::High) {
            return Err(HardwareError::SrlatchForbiddenInput);
        }

        stabilize(
            || (self.q.get(), self.q_bar.get()),
            || {
                let new_q_bar = self
                    .nor1
                    .conduct(&[set, Signal::from(self.q.get())])
                    .expect("a NOR gate with driven inputs cannot fail")[0];
                let new_q = self
                    .nor2
                    .conduct(&[reset, new_q_bar])
                    .expect("a NOR gate with driven inputs cannot fail")[0];

                self.q_bar
                    .set(Bit::try_from(new_q_bar).expect("NOR gates always drive their output"));
                self.q
                    .set(Bit::try_from(new_q).expect("NOR gates always drive their output"));
            },
            MAX_STABILIZATION_ITERATIONS,
        )?;

        Ok(vec![
            Signal::from(self.q.get()),
            Signal::from(self.q_bar.get()),
        ])
    }

    fn transistor_count(&self) -> usize {
        self.nor1.transistor_count() + self.nor2.transistor_count()
    }
}

/// D latch: transparent while `enable` is HIGH, holds otherwise.
#[derive(Debug, Default)]
pub struct DLatch {
    sr_latch: SRLatch,
    not_data: NotGate,
    and_set: AndGate,
    and_reset: AndGate,
}

impl DLatch {
    /// The stored output.
    pub fn q(&self) -> Bit {
        self.sr_latch.q()
    }

    /// The complement of the stored output.
    pub fn q_bar(&self) -> Bit {
        self.sr_latch.q_bar()
    }
}

impl Component for DLatch {
    /// Inputs:
    ///
    /// - 0: data
    /// - 1: enable
    ///
    /// Outputs:
    ///
    /// - 0: q
    /// - 1: q_bar
    ///
    /// Follow `data` while `enable` is High, hold otherwise.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [data, enable] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let not_data = self.not_data.conduct(&[*data])?;
        let set = self.and_set.conduct(&[*data, *enable])?;
        let reset = self.and_reset.conduct(&[not_data[0], *enable])?;

        self.sr_latch.conduct(&[set[0], reset[0]])
    }

    fn transistor_count(&self) -> usize {
        self.sr_latch.transistor_count()
            + self.not_data.transistor_count()
            + self.and_set.transistor_count()
            + self.and_reset.transistor_count()
    }
}

/// Master-slave D flip-flop, updated on the rising edge of `clock`.
#[derive(Debug, Default)]
pub struct DFlipFlop {
    master_latch: DLatch,
    slave_latch: DLatch,
    not_clock: NotGate,
}

impl DFlipFlop {
    /// The stored output.
    pub fn q(&self) -> Bit {
        self.slave_latch.q()
    }

    /// The complement of the stored output.
    pub fn q_bar(&self) -> Bit {
        self.slave_latch.q_bar()
    }
}

impl Component for DFlipFlop {
    /// Inputs:
    ///
    /// - 0: data
    /// - 1: clock
    ///
    /// Outputs:
    ///
    /// - 0: q
    /// - 1: q_bar
    ///
    /// Capture `data` on the rising edge of `clock`.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [data, clock] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let not_clock = self.not_clock.conduct(&[*clock])?;
        let master_output = self.master_latch.conduct(&[*data, not_clock[0]])?;

        self.slave_latch.conduct(&[master_output[0], *clock])
    }

    fn transistor_count(&self) -> usize {
        self.master_latch.transistor_count()
            + self.slave_latch.transistor_count()
            + self.not_clock.transistor_count()
    }
}

/// D flip-flop whose writes are gated by `save`.
#[derive(Debug, Default)]
pub struct DFlipFlopSave {
    master_latch: DLatch,
    slave_latch: DLatch,
    not_clock: NotGate,
    mux: Mux2x1,
}

impl DFlipFlopSave {
    /// The stored output.
    pub fn q(&self) -> Bit {
        self.slave_latch.q()
    }

    /// The complement of the stored output.
    pub fn q_bar(&self) -> Bit {
        self.slave_latch.q_bar()
    }
}

impl Component for DFlipFlopSave {
    /// Inputs:
    ///
    /// - 0: data
    /// - 1: clock
    /// - 2: save
    ///
    /// Outputs:
    ///
    /// - 0: q
    /// - 1: q_bar
    ///
    /// Capture `data` on the rising edge only if `save` is High. When
    /// `save` is Low, the slave is fed its own output, so the rising
    /// edge keeps the previous value.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [data, clock, save] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 3,
                actual: inputs.len(),
            });
        };

        let not_clock = self.not_clock.conduct(&[*clock])?;
        let master_output = self.master_latch.conduct(&[*data, not_clock[0]])?;
        let selected_input =
            self.mux
                .conduct(&[Signal::from(self.slave_latch.q()), master_output[0], *save])?;

        self.slave_latch.conduct(&[selected_input[0], *clock])
    }

    fn transistor_count(&self) -> usize {
        self.master_latch.transistor_count()
            + self.slave_latch.transistor_count()
            + self.not_clock.transistor_count()
            + self.mux.transistor_count()
    }
}

/// D flip-flop with save, whose outputs are gated by `load`.
#[derive(Debug, Default)]
pub struct DFlipFlopSaveLoad {
    flip_flop: DFlipFlopSave,
    nmos_q: Nmos,
    nmos_q_bar: Nmos,
}

impl DFlipFlopSaveLoad {
    /// The stored output.
    pub fn q(&self) -> Bit {
        self.flip_flop.q()
    }

    /// The complement of the stored output.
    pub fn q_bar(&self) -> Bit {
        self.flip_flop.q_bar()
    }
}

impl Component for DFlipFlopSaveLoad {
    /// Inputs:
    ///
    /// - 0: data
    /// - 1: clock
    /// - 2: save
    /// - 3: load
    ///
    /// Outputs:
    ///
    /// - 0: q, floating when `load` is Low
    /// - 1: q_bar, floating when `load` is Low
    ///
    /// Behave as `DFlipFlopSave`; float the outputs when `load` is Low.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [data, clock, save, load] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 4,
                actual: inputs.len(),
            });
        };

        let outputs = self.flip_flop.conduct(&[*data, *clock, *save])?;
        let q = self.nmos_q.conduct(&[*load, outputs[0]])?;
        let q_bar = self.nmos_q_bar.conduct(&[*load, outputs[1]])?;

        Ok(vec![q[0], q_bar[0]])
    }

    fn transistor_count(&self) -> usize {
        self.flip_flop.transistor_count()
            + self.nmos_q.transistor_count()
            + self.nmos_q_bar.transistor_count()
    }
}

/// Ring counter shifting a single HIGH bit on each rising clock edge.
#[derive(Debug)]
pub struct OneHotCounter6Bits {
    flip_flops: [DFlipFlop; COUNTER_BITS],
}

impl OneHotCounter6Bits {
    /// Create a counter whose HIGH bit starts at `initial_position`.
    pub fn new(initial_position: usize) -> Self {
        assert!(
            initial_position < COUNTER_BITS,
            "initial_position must be within [0, {}].",
            COUNTER_BITS - 1
        );

        let counter = Self {
            flip_flops: <[DFlipFlop; COUNTER_BITS]>::default(),
        };

        // Force the starting bit HIGH with a LOW->HIGH clock sequence.
        counter.flip_flops[initial_position]
            .conduct(&[Signal::from(Bit::High), Signal::from(Bit::Low)])
            .expect("a flip-flop with driven inputs cannot fail");
        counter.flip_flops[initial_position]
            .conduct(&[Signal::from(Bit::High), Signal::from(Bit::High)])
            .expect("a flip-flop with driven inputs cannot fail");

        counter
    }

    /// The current position of the single HIGH bit.
    pub fn state(&self) -> [Bit; COUNTER_BITS] {
        self.flip_flops.each_ref().map(|flip_flop| flip_flop.q())
    }
}

impl Default for OneHotCounter6Bits {
    fn default() -> Self {
        Self::new(COUNTER_BITS - 1)
    }
}

impl Component for OneHotCounter6Bits {
    /// Inputs:
    ///
    /// - 0: clock
    ///
    /// Outputs:
    ///
    /// - 0..6: the ring state, holding exactly one High bit.
    ///
    /// Shift the HIGH bit by one position on the rising edge.
    ///
    /// The ring is wired like the real circuit: each flip-flop's data
    /// input is the previous flip-flop's output, and the last output is
    /// fed back to the first one. The edge-triggered flip-flops break
    /// the feedback loop, so a single propagation pass over the ring
    /// computes the new state.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [clock] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 1,
                actual: inputs.len(),
            });
        };

        // The feedback loop is broken by the clock edge, so the shift
        // reads the last flip-flop *before* it advances.
        let wrapped_input = Signal::from(self.flip_flops[COUNTER_BITS - 1].q());
        let mut previous_q = self.flip_flops[0].conduct(&[wrapped_input, *clock])?[0];

        for flip_flop in self.flip_flops.iter().skip(1) {
            previous_q = flip_flop.conduct(&[previous_q, *clock])?[0];
        }

        Ok(self.state().map(Signal::from).to_vec())
    }

    fn transistor_count(&self) -> usize {
        self.flip_flops
            .iter()
            .map(|flip_flop| flip_flop.transistor_count())
            .sum()
    }
}

#[cfg(test)]
mod srlatch_tests {
    use super::*;
    use Bit::{High, Low};

    #[test]
    fn test_initial_state_is_reset() {
        assert_eq!(SRLatch::default().compute(&[Low, Low]), Ok(vec![Low, High]));
    }

    #[test]
    fn test_set_then_hold() {
        let latch = SRLatch::default();
        assert_eq!(latch.compute(&[High, Low]), Ok(vec![High, Low]));
        assert_eq!(latch.compute(&[Low, Low]), Ok(vec![High, Low]));
    }

    #[test]
    fn test_reset_then_hold() {
        let latch = SRLatch::default();
        latch.compute(&[High, Low]).expect("set failed");
        assert_eq!(latch.compute(&[Low, High]), Ok(vec![Low, High]));
        assert_eq!(latch.compute(&[Low, Low]), Ok(vec![Low, High]));
    }

    #[test]
    fn test_set_and_reset_high_is_forbidden() {
        assert_eq!(
            SRLatch::default().conduct(&[Signal::from(High), Signal::from(High)]),
            Err(HardwareError::SrlatchForbiddenInput)
        );
    }

    #[test]
    fn test_floating_input_is_rejected() {
        assert_eq!(
            SRLatch::default().conduct(&[Signal::HighImpedance, Signal::from(Low)]),
            Err(HardwareError::InvalidBit)
        );
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(SRLatch::default().transistor_count(), 8);
    }
}

#[cfg(test)]
mod dlatch_tests {
    use super::*;
    use Bit::{High, Low};

    #[test]
    fn test_follows_data_while_enabled_and_holds() {
        let latch = DLatch::default();

        let sequence = [
            ([Low, Low], [Low, High]),
            ([High, Low], [Low, High]),
            ([Low, High], [Low, High]),
            ([High, High], [High, Low]),
            ([Low, Low], [High, Low]),
            ([High, Low], [High, Low]),
            ([High, High], [High, Low]),
            ([Low, High], [Low, High]),
        ];

        for (inputs, expected) in sequence {
            assert_eq!(latch.compute(&inputs), Ok(expected.to_vec()));
        }
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(DLatch::default().transistor_count(), 22);
    }
}

#[cfg(test)]
mod dflipflop_tests {
    use super::*;
    use Bit::{High, Low};

    #[test]
    fn test_updates_only_on_rising_edge() {
        let flip_flop = DFlipFlop::default();

        let sequence = [
            ([Low, Low], [Low, High]),
            ([High, Low], [Low, High]), // data prepared while clock is Low
            ([Low, Low], [Low, High]),
            ([Low, High], [Low, High]),  // rising edge with data 0
            ([High, High], [Low, High]), // no edge, no update
            ([High, Low], [Low, High]),
            ([High, High], [High, Low]), // rising edge with data 1
            ([Low, High], [High, Low]),
            ([Low, Low], [High, Low]),
            ([Low, High], [Low, High]),
        ];

        for (inputs, expected) in sequence {
            assert_eq!(flip_flop.compute(&inputs), Ok(expected.to_vec()));
        }
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(DFlipFlop::default().transistor_count(), 46);
    }
}

#[cfg(test)]
mod dflipflopsave_tests {
    use super::*;
    use Bit::{High, Low};

    #[test]
    fn test_save_low_never_writes() {
        let flip_flop = DFlipFlopSave::default();

        for data in [Low, High, Low, Low, High, High, Low, Low] {
            for clock in [Low, High] {
                assert_eq!(flip_flop.compute(&[data, clock, Low]), Ok(vec![Low, High]));
            }
        }
    }

    #[test]
    fn test_save_high_behaves_like_a_flip_flop() {
        let flip_flop = DFlipFlopSave::default();

        let sequence = [
            ([Low, Low, High], [Low, High]),
            ([High, Low, High], [Low, High]),
            ([Low, Low, High], [Low, High]),
            ([Low, High, High], [Low, High]),
            ([High, High, High], [Low, High]),
            ([High, Low, High], [Low, High]),
            ([High, High, High], [High, Low]),
            ([Low, High, High], [High, Low]),
            ([Low, Low, High], [High, Low]),
            ([Low, High, High], [Low, High]),
        ];

        for (inputs, expected) in sequence {
            assert_eq!(flip_flop.compute(&inputs), Ok(expected.to_vec()));
        }
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(DFlipFlopSave::default().transistor_count(), 66);
    }
}

#[cfg(test)]
mod dflipflopsaveload_tests {
    use super::*;
    use Bit::{High, Low};

    #[test]
    fn test_load_low_floats_the_outputs() {
        let outputs = DFlipFlopSaveLoad::default()
            .conduct(&[Signal::from(Low); 4])
            .expect("computation failed");

        assert_eq!(outputs, vec![Signal::HighImpedance; 2]);
    }

    #[test]
    fn test_load_low_is_not_computable() {
        // `compute` requires every output to be driven.
        assert_eq!(
            DFlipFlopSaveLoad::default().compute(&[Low, Low, Low, Low]),
            Err(HardwareError::InvalidBit)
        );
    }

    #[test]
    fn test_load_high_exposes_the_state() {
        let flip_flop = DFlipFlopSaveLoad::default();
        assert_eq!(
            flip_flop.compute(&[High, Low, High, High]),
            Ok(vec![Low, High])
        );
        assert_eq!(
            flip_flop.compute(&[High, High, High, High]),
            Ok(vec![High, Low])
        );
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(DFlipFlopSaveLoad::default().transistor_count(), 68);
    }
}

#[cfg(test)]
mod onehotcounter6bits_tests {
    use super::*;
    use Bit::{High, Low};

    #[test]
    fn test_initial_state() {
        assert_eq!(
            OneHotCounter6Bits::default().state(),
            [Low, Low, Low, Low, Low, High]
        );
    }

    #[test]
    fn test_custom_initial_position() {
        assert_eq!(
            OneHotCounter6Bits::new(2).state(),
            [Low, Low, High, Low, Low, Low]
        );
    }

    #[test]
    #[should_panic(expected = "initial_position must be within [0, 5]")]
    fn test_rejects_invalid_initial_position() {
        let _ = OneHotCounter6Bits::new(COUNTER_BITS);
    }

    #[test]
    fn test_rotation() {
        let counter = OneHotCounter6Bits::default();

        let expected = [
            [High, Low, Low, Low, Low, Low],
            [Low, High, Low, Low, Low, Low],
            [Low, Low, High, Low, Low, Low],
            [Low, Low, Low, High, Low, Low],
            [Low, Low, Low, Low, High, Low],
            [Low, Low, Low, Low, Low, High],
            [High, Low, Low, Low, Low, Low],
        ];

        for state in expected {
            counter.compute(&[Low]).expect("clock Low failed");
            assert_eq!(counter.compute(&[High]), Ok(state.to_vec()));
        }
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(OneHotCounter6Bits::default().transistor_count(), 276);
    }
}
