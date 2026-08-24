//! Multiplexers built on top of the logic gates from [`crate::gates`].
//!
//! A 2-to-1 multiplexer forwards input `a` when `select` is `Low` and
//! input `b` when `select` is `High`:
//!
//!     output = (a AND NOT select) OR (b AND select)
//!
//! The 8-bit variant simply places eight of them in parallel.

use crate::gates::{AndGate, NotGate, OrGate};
use crate::hardware::{Component, HardwareError, Signal};

/// Two-to-one multiplexer: 1 NOT, 2 AND, 1 OR.
#[derive(Debug, Default)]
pub struct Mux2x1 {
    not_select: NotGate,
    and_a: AndGate,
    and_b: AndGate,
    or_output: OrGate,
}

impl Component for Mux2x1 {
    /// Inputs:
    ///
    /// - 0: a
    /// - 1: b
    /// - 2: select
    ///
    /// Outputs:
    ///
    /// - 0: `a` when `select` is Low, `b` when `select` is High.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [a, b, select] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 3,
                actual: inputs.len(),
            });
        };

        let not_select = self.not_select.conduct(&[*select])?;
        let path_a = self.and_a.conduct(&[*a, not_select[0]])?;
        let path_b = self.and_b.conduct(&[*b, *select])?;

        self.or_output.conduct(&[path_a[0], path_b[0]])
    }

    fn transistor_count(&self) -> usize {
        self.not_select.transistor_count()
            + self.and_a.transistor_count()
            + self.and_b.transistor_count()
            + self.or_output.transistor_count()
    }
}

/// Eight two-to-one multiplexers in parallel, one per bit.
#[derive(Debug, Default)]
pub struct Mux8bits2x1 {
    muxes: [Mux2x1; 8],
}

impl Component for Mux8bits2x1 {
    /// Inputs:
    ///
    /// - 0..8:  a, least-significant bit first
    /// - 8..16: b, least-significant bit first
    /// - 16:    select
    ///
    /// Outputs:
    ///
    /// - 0..8: the selected operand, least-significant bit first.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        if inputs.len() != 17 {
            return Err(HardwareError::InvalidInputCount {
                expected: 17,
                actual: inputs.len(),
            });
        }

        let select = inputs[16];
        let mut outputs = Vec::with_capacity(8);

        for (mux, (a, b)) in self
            .muxes
            .iter()
            .zip(inputs[..8].iter().zip(inputs[8..16].iter()))
        {
            let output = mux.conduct(&[*a, *b, select])?;
            outputs.push(output[0]);
        }

        Ok(outputs)
    }

    fn transistor_count(&self) -> usize {
        self.muxes.iter().map(|mux| mux.transistor_count()).sum()
    }
}

#[cfg(test)]
mod mux2x1_tests {
    use super::*;
    use crate::hardware::Bit;

    #[test]
    fn test_truth_table() {
        let mux = Mux2x1::default();

        for a in [Bit::Low, Bit::High] {
            for b in [Bit::Low, Bit::High] {
                for select in [Bit::Low, Bit::High] {
                    let expected = if select == Bit::High { b } else { a };
                    assert_eq!(mux.compute(&[a, b, select]), Ok(vec![expected]));
                }
            }
        }
    }

    #[test]
    fn test_rejects_wrong_input_count() {
        assert_eq!(
            Mux2x1::default().conduct(&[Signal::HighImpedance, Signal::HighImpedance]),
            Err(HardwareError::InvalidInputCount {
                expected: 3,
                actual: 2,
            })
        );
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(Mux2x1::default().transistor_count(), 20);
    }
}

#[cfg(test)]
mod mux8bits2x1_tests {
    use super::*;
    use crate::hardware::Bit;
    use Bit::{High, Low};

    const A: [Bit; 8] = [Low, High, Low, High, Low, High, Low, High];
    const B: [Bit; 8] = [High, Low, High, Low, High, Low, High, Low];

    fn inputs(a: [Bit; 8], b: [Bit; 8], select: Bit) -> Vec<Bit> {
        a.into_iter()
            .chain(b)
            .chain(std::iter::once(select))
            .collect()
    }

    #[test]
    fn test_selects_a_when_low() {
        let outputs = Mux8bits2x1::default()
            .compute(&inputs(A, B, Bit::Low))
            .expect("Mux8bits2x1 computation failed");

        assert_eq!(outputs, A);
    }

    #[test]
    fn test_selects_b_when_high() {
        let outputs = Mux8bits2x1::default()
            .compute(&inputs(A, B, Bit::High))
            .expect("Mux8bits2x1 computation failed");

        assert_eq!(outputs, B);
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(Mux8bits2x1::default().transistor_count(), 160);
    }
}
