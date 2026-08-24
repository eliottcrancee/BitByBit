//! The lowest-level primitives of the project.
//!
//! Everything above this module is built exclusively from [`Pmos`] and
//! [`Nmos`] transistors through the [`Component`] trait:
//!
//! * [`Bit`] — a binary logic level.
//! * [`Signal`] — a wire state: driven low/high, or floating
//!   (high impedance).
//! * [`wire`] / [`bus8`] — resolution of several drivers on the same
//!   line, detecting short circuits.
//! * [`stabilize`] — fixed-point iteration for feedback loops
//!   (latches).
//! * [`bits_to_int`] / [`int_to_bits`] — conversion helpers; all
//!   multi-bit values in the project are least-significant bit first.

/// A binary logic level.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Bit {
    /// Logic 0.
    #[default]
    Low,
    /// Logic 1.
    High,
}

/// The electrical state of a wire.
///
/// A tri-state model: a wire is either driven to a [`Bit`] level by a
/// transistor, or left floating ([`Signal::HighImpedance`]) when no
/// driver is active. Floating wires are resolved by [`wire`] when
/// several drivers meet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Signal {
    /// A transistor actively drives the wire to this level.
    Driven(Bit),
    /// No transistor drives the wire.
    HighImpedance,
}

/// Errors raised while propagating signals through the circuit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareError {
    /// Two drivers pulled the same wire to opposite levels.
    ShortCircuit { first: Bit, second: Bit },
    /// A component received the wrong number of inputs.
    InvalidInputCount { expected: usize, actual: usize },
    /// A high-impedance signal was read where a logic level was required.
    InvalidBit,
    /// Both SR latch inputs were HIGH at the same time.
    SrlatchForbiddenInput,
    /// A feedback loop did not converge within the iteration budget.
    UnstableCircuit { iterations: usize },
}

impl From<Bit> for Signal {
    fn from(bit: Bit) -> Self {
        Signal::Driven(bit)
    }
}

impl From<Bit> for bool {
    fn from(bit: Bit) -> Self {
        match bit {
            Bit::Low => false,
            Bit::High => true,
        }
    }
}

impl TryFrom<Signal> for Bit {
    type Error = HardwareError;

    fn try_from(signal: Signal) -> Result<Self, Self::Error> {
        match signal {
            Signal::Driven(bit) => Ok(bit),
            Signal::HighImpedance => Err(HardwareError::InvalidBit),
        }
    }
}

/// The common interface of every piece of hardware in the project.
///
/// A component propagates input signals to its outputs through its
/// internal transistors. Combinational components are pure functions of
/// their inputs; sequential components also update their internal state
/// (through interior mutability, since `conduct` takes `&self`).
pub trait Component {
    /// Propagate the input signals and return the output signals.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError>;

    /// Number of transistors this component is built from,
    /// sub-components included.
    fn transistor_count(&self) -> usize;

    /// Convenience wrapper over [`Component::conduct`] for combinational
    /// use: takes logic levels in, requires every output to be driven.
    fn compute(&self, inputs: &[Bit]) -> Result<Vec<Bit>, HardwareError> {
        let signals: Vec<Signal> = inputs.iter().copied().map(Signal::from).collect();
        match self.conduct(&signals) {
            Ok(signals) => signals
                .into_iter()
                .map(Bit::try_from)
                .collect::<Result<Vec<Bit>, HardwareError>>(),
            Err(e) => Err(e),
        }
    }
}

/// A P-channel MOSFET: conducts when its gate is LOW.
#[derive(Debug, Default)]
pub struct Pmos;

impl Component for Pmos {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [gate, source] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        match gate {
            Signal::Driven(Bit::Low) => Ok(vec![*source]),
            Signal::Driven(Bit::High) | Signal::HighImpedance => Ok(vec![Signal::HighImpedance]),
        }
    }

    fn transistor_count(&self) -> usize {
        1
    }
}

/// An N-channel MOSFET: conducts when its gate is HIGH.
#[derive(Debug, Default)]
pub struct Nmos;

impl Component for Nmos {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [gate, source] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        match gate {
            Signal::Driven(Bit::High) => Ok(vec![*source]),
            Signal::Driven(Bit::Low) | Signal::HighImpedance => Ok(vec![Signal::HighImpedance]),
        }
    }

    fn transistor_count(&self) -> usize {
        1
    }
}

/// Resolve the state of a wire driven by several transistors.
///
/// Floating drivers have no effect and agreeing drivers are merged; two
/// drivers pulling in opposite directions is a short circuit.
///
/// Inputs 0 (gate) and 1 (source); output 0.
pub fn wire<I>(signals: I) -> Result<Signal, HardwareError>
where
    I: IntoIterator<Item = Signal>,
{
    let mut result = Signal::HighImpedance;

    for signal in signals {
        match (result, signal) {
            // Nothing is currently driving the wire.
            (Signal::HighImpedance, signal) => {
                result = signal;
            }

            // High impedance has no effect on the wire.
            (_, Signal::HighImpedance) => {}

            // Multiple drivers agree.
            (Signal::Driven(a), Signal::Driven(b)) if a == b => {}

            // Two drivers disagree: short circuit.
            (Signal::Driven(a), Signal::Driven(b)) => {
                return Err(HardwareError::ShortCircuit {
                    first: a,
                    second: b,
                });
            }
        }
    }

    Ok(result)
}

/// Resolve an 8-bit bus driven by several tri-state drivers, bit per bit.
pub fn bus8<I>(buses: I) -> Result<[Signal; 8], HardwareError>
where
    I: IntoIterator<Item = [Signal; 8]>,
{
    let mut result = [Signal::HighImpedance; 8];

    for bus in buses {
        for (index, signal) in bus.into_iter().enumerate() {
            result[index] = wire([result[index], signal])?;
        }
    }

    Ok(result)
}

/// Pack up to eight bits into a byte.
///
/// # Panics
///
/// Panics when `bits` is longer than eight.
pub fn bits_to_int(bits: &[Bit], msb_first: bool) -> u8 {
    assert!(
        bits.len() <= 8,
        "The project only manipulates 8-bit values."
    );

    let mut result: u8 = 0;

    if msb_first {
        for bit in bits {
            result = (result << 1) | *bit as u8;
        }
    } else {
        for (index, bit) in bits.iter().enumerate() {
            result |= (*bit as u8) << index;
        }
    }

    result
}

/// Expand a byte into eight bits, least significant bit first.
pub fn int_to_bits(value: u8) -> [Bit; 8] {
    core::array::from_fn(|index| {
        if (value >> index) & 1 == 1 {
            Bit::High
        } else {
            Bit::Low
        }
    })
}

/// Run a feedback loop until its state stops changing.
///
/// Models the analog settling of a feedback circuit (a latch): the
/// update closure is applied repeatedly until two consecutive states
/// match. A loop that keeps oscillating is reported as unstable.
///
/// # Panics
///
/// Panics when `max_iterations` is zero.
pub fn stabilize<State, GetState, UpdateState>(
    get_state: GetState,
    mut update_state: UpdateState,
    max_iterations: usize,
) -> Result<(), HardwareError>
where
    State: PartialEq,
    GetState: Fn() -> State,
    UpdateState: FnMut(),
{
    assert!(
        max_iterations > 0,
        "max_iterations must be strictly positive."
    );

    let mut previous = get_state();

    for _ in 0..max_iterations {
        update_state();
        let current = get_state();

        if current == previous {
            return Ok(());
        }

        previous = current;
    }

    Err(HardwareError::UnstableCircuit {
        iterations: max_iterations,
    })
}

#[cfg(test)]
mod pmos_tests {
    use super::*;

    #[test]
    fn test_pmos_conduct() {
        let pmos = Pmos;

        // Gate Low: PMOS is ON.
        assert_eq!(
            pmos.conduct(&[Signal::Driven(Bit::Low), Signal::Driven(Bit::High)]),
            Ok(vec![Signal::Driven(Bit::High)])
        );

        assert_eq!(
            pmos.conduct(&[Signal::Driven(Bit::Low), Signal::Driven(Bit::Low)]),
            Ok(vec![Signal::Driven(Bit::Low)])
        );

        assert_eq!(
            pmos.conduct(&[Signal::Driven(Bit::Low), Signal::HighImpedance]),
            Ok(vec![Signal::HighImpedance])
        );

        // Gate High: PMOS is OFF.
        assert_eq!(
            pmos.conduct(&[Signal::Driven(Bit::High), Signal::Driven(Bit::High)]),
            Ok(vec![Signal::HighImpedance])
        );

        assert_eq!(
            pmos.conduct(&[Signal::Driven(Bit::High), Signal::Driven(Bit::Low)]),
            Ok(vec![Signal::HighImpedance])
        );

        assert_eq!(
            pmos.conduct(&[Signal::Driven(Bit::High), Signal::HighImpedance]),
            Ok(vec![Signal::HighImpedance])
        );

        // Floating gate: PMOS is OFF.
        assert_eq!(
            pmos.conduct(&[Signal::HighImpedance, Signal::Driven(Bit::High)]),
            Ok(vec![Signal::HighImpedance])
        );
    }

    #[test]
    fn test_pmos_compute() {
        let pmos = Pmos;

        assert_eq!(pmos.compute(&[Bit::Low, Bit::Low]), Ok(vec![Bit::Low]));
        assert_eq!(pmos.compute(&[Bit::Low, Bit::High]), Ok(vec![Bit::High]));

        assert_eq!(pmos.transistor_count(), 1);
    }
}

#[cfg(test)]
mod nmos_tests {
    use super::*;

    #[test]
    fn test_nmos_conduct() {
        let nmos = Nmos;

        // Gate Low: NMOS is OFF.
        assert_eq!(
            nmos.conduct(&[Signal::Driven(Bit::Low), Signal::Driven(Bit::High)]),
            Ok(vec![Signal::HighImpedance])
        );

        assert_eq!(
            nmos.conduct(&[Signal::Driven(Bit::Low), Signal::Driven(Bit::Low)]),
            Ok(vec![Signal::HighImpedance])
        );

        assert_eq!(
            nmos.conduct(&[Signal::Driven(Bit::Low), Signal::HighImpedance]),
            Ok(vec![Signal::HighImpedance])
        );

        // Gate High: NMOS is ON.
        assert_eq!(
            nmos.conduct(&[Signal::Driven(Bit::High), Signal::Driven(Bit::High)]),
            Ok(vec![Signal::Driven(Bit::High)])
        );

        assert_eq!(
            nmos.conduct(&[Signal::Driven(Bit::High), Signal::Driven(Bit::Low)]),
            Ok(vec![Signal::Driven(Bit::Low)])
        );

        assert_eq!(
            nmos.conduct(&[Signal::Driven(Bit::High), Signal::HighImpedance]),
            Ok(vec![Signal::HighImpedance])
        );

        // Floating gate: NMOS is OFF.
        assert_eq!(
            nmos.conduct(&[Signal::HighImpedance, Signal::Driven(Bit::High)]),
            Ok(vec![Signal::HighImpedance])
        );
    }

    #[test]
    fn test_nmos_compute() {
        let nmos = Nmos;

        assert_eq!(
            nmos.compute(&[Bit::Low, Bit::Low]),
            Err(HardwareError::InvalidBit),
        );

        assert_eq!(
            nmos.compute(&[Bit::Low, Bit::High]),
            Err(HardwareError::InvalidBit)
        );

        assert_eq!(nmos.compute(&[Bit::High, Bit::Low]), Ok(vec![Bit::Low]));
        assert_eq!(nmos.compute(&[Bit::High, Bit::High]), Ok(vec![Bit::High]));

        assert_eq!(nmos.transistor_count(), 1);
    }
}

#[cfg(test)]
mod wire_tests {
    use super::*;

    #[test]
    fn test_empty_wire() {
        assert_eq!(wire([]), Ok(Signal::HighImpedance));
    }

    #[test]
    fn test_wire_only_high_impedance() {
        assert_eq!(
            wire([Signal::HighImpedance, Signal::HighImpedance,]),
            Ok(Signal::HighImpedance)
        );
    }

    #[test]
    fn test_wire_single_low_driver() {
        assert_eq!(
            wire([Signal::Driven(Bit::Low), Signal::HighImpedance,]),
            Ok(Signal::Driven(Bit::Low))
        );
    }

    #[test]
    fn test_wire_single_high_driver() {
        assert_eq!(
            wire([Signal::Driven(Bit::High), Signal::HighImpedance,]),
            Ok(Signal::Driven(Bit::High))
        );
    }

    #[test]
    fn test_wire_multiple_low_drivers() {
        assert_eq!(
            wire([Signal::Driven(Bit::Low), Signal::Driven(Bit::Low),]),
            Ok(Signal::Driven(Bit::Low))
        );
    }

    #[test]
    fn test_wire_multiple_high_drivers() {
        assert_eq!(
            wire([Signal::Driven(Bit::High), Signal::Driven(Bit::High),]),
            Ok(Signal::Driven(Bit::High))
        );
    }

    #[test]
    fn test_wire_low_high_conflict() {
        assert_eq!(
            wire([Signal::Driven(Bit::Low), Signal::Driven(Bit::High),]),
            Err(HardwareError::ShortCircuit {
                first: Bit::Low,
                second: Bit::High,
            })
        );
    }

    #[test]
    fn test_wire_high_low_conflict() {
        assert_eq!(
            wire([Signal::Driven(Bit::High), Signal::Driven(Bit::Low),]),
            Err(HardwareError::ShortCircuit {
                first: Bit::High,
                second: Bit::Low,
            })
        );
    }

    #[test]
    fn test_wire_with_multiple_high_impedance() {
        assert_eq!(
            wire([
                Signal::HighImpedance,
                Signal::Driven(Bit::Low),
                Signal::HighImpedance,
                Signal::Driven(Bit::Low),
                Signal::HighImpedance,
            ]),
            Ok(Signal::Driven(Bit::Low))
        );
    }

    #[test]
    fn test_wire_works_with_iterator() {
        let signals = vec![
            Signal::HighImpedance,
            Signal::Driven(Bit::High),
            Signal::HighImpedance,
            Signal::Driven(Bit::High),
        ];

        assert_eq!(wire(signals), Ok(Signal::Driven(Bit::High)));
    }

    #[test]
    fn test_wire_detects_conflict_after_high_impedance() {
        assert_eq!(
            wire([
                Signal::HighImpedance,
                Signal::Driven(Bit::Low),
                Signal::HighImpedance,
                Signal::Driven(Bit::High),
            ]),
            Err(HardwareError::ShortCircuit {
                first: Bit::Low,
                second: Bit::High,
            })
        );
    }
}

#[cfg(test)]
mod bus8_tests {
    use super::*;

    #[test]
    fn test_bus8_no_bus_floats() {
        assert_eq!(bus8([]), Ok([Signal::HighImpedance; 8]));
    }

    #[test]
    fn test_bus8_merges_drivers() {
        let bus_a = [
            Signal::Driven(Bit::High),
            Signal::HighImpedance,
            Signal::HighImpedance,
            Signal::Driven(Bit::Low),
            Signal::HighImpedance,
            Signal::HighImpedance,
            Signal::HighImpedance,
            Signal::HighImpedance,
        ];
        let bus_b = [
            Signal::HighImpedance,
            Signal::Driven(Bit::Low),
            Signal::HighImpedance,
            Signal::HighImpedance,
            Signal::HighImpedance,
            Signal::HighImpedance,
            Signal::Driven(Bit::High),
            Signal::HighImpedance,
        ];

        assert_eq!(
            bus8([bus_a, bus_b]),
            Ok([
                Signal::Driven(Bit::High),
                Signal::Driven(Bit::Low),
                Signal::HighImpedance,
                Signal::Driven(Bit::Low),
                Signal::HighImpedance,
                Signal::HighImpedance,
                Signal::Driven(Bit::High),
                Signal::HighImpedance,
            ])
        );
    }

    #[test]
    fn test_bus8_detects_conflicts() {
        let bus_a = [Signal::Driven(Bit::Low); 8];
        let bus_b = [Signal::Driven(Bit::High); 8];

        assert_eq!(
            bus8([bus_a, bus_b]),
            Err(HardwareError::ShortCircuit {
                first: Bit::Low,
                second: Bit::High,
            })
        );
    }
}

#[cfg(test)]
mod bits_to_int_tests {
    use super::*;
    use Bit::{High, Low};

    #[test]
    fn test_empty_is_zero() {
        assert_eq!(bits_to_int(&[], false), 0);
    }

    #[test]
    fn test_lsb_first_by_default() {
        assert_eq!(bits_to_int(&[High, Low, High], false), 5);
        assert_eq!(bits_to_int(&[High; 8], false), 255);
    }

    #[test]
    fn test_msb_first() {
        assert_eq!(bits_to_int(&[High, Low, High], true), 0b101);
        assert_eq!(
            bits_to_int(&[Low, Low, Low, Low, High, Low, High, Low], true),
            0x0A
        );
    }
}

#[cfg(test)]
mod stabilize_tests {
    use super::*;
    use std::cell::Cell;

    #[test]
    fn test_converges() {
        let state = Cell::new(false);

        let outcome = stabilize(
            || state.get(),
            || {
                state.set(true);
            },
            10,
        );

        assert_eq!(outcome, Ok(()));
        assert!(state.get());
    }

    #[test]
    fn test_already_stable() {
        let outcome = stabilize(
            || 42,
            || {
                // The circuit does not evolve at all.
            },
            10,
        );

        assert_eq!(outcome, Ok(()));
    }

    #[test]
    fn test_oscillating_circuit_is_rejected() {
        let state = Cell::new(false);

        let outcome = stabilize(
            || state.get(),
            || {
                state.set(!state.get());
            },
            10,
        );

        assert_eq!(
            outcome,
            Err(HardwareError::UnstableCircuit { iterations: 10 })
        );
    }

    #[test]
    #[should_panic(expected = "max_iterations must be strictly positive")]
    fn test_rejects_zero_iterations() {
        let _ = stabilize(|| 0, || {}, 0);
    }
}
