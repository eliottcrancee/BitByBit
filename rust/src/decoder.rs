//! Binary decoders built on top of the logic gates from [`crate::gates`].
//!
//! A decoder activates exactly one output line for a given binary input
//! value. For `Decoder2to4`, the output index activated by inputs
//! `(a, b)` is `2 * a + b`, meaning `a` is the most significant bit of
//! the pair.
//!
//! `Decoder4to16` cascades two `Decoder2to4` and ANDs their outputs.
//! Its 4-bit input is given least significant bit first, so the
//! activated output index is
//! `inputs[0] + 2 * inputs[1] + 4 * inputs[2] + 8 * inputs[3]`.

use crate::gates::{AndGate, NotGate};
use crate::hardware::{Component, HardwareError, Signal};

/// Two-to-four decoder: 2 NOT, 4 AND.
#[derive(Debug, Default)]
pub struct Decoder2to4 {
    not_a: NotGate,
    not_b: NotGate,
    and_gates: [AndGate; 4],
}

impl Component for Decoder2to4 {
    const INPUTS: usize = 2;
    const OUTPUTS: usize = 4;

    /// Inputs:
    ///
    /// - 0: a, most significant bit
    /// - 1: b, least significant bit
    ///
    /// Outputs:
    ///
    /// - exactly one output is Driven(High): the one at index `2 * a + b`.
    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let mut not_a = [Signal::HighImpedance];
        self.not_a.conduct_into(&[*a], &mut not_a)?;
        let mut not_b = [Signal::HighImpedance];
        self.not_b.conduct_into(&[*b], &mut not_b)?;

        self.and_gates[0].conduct_into(&[not_a[0], not_b[0]], &mut outputs[0..1])?;
        self.and_gates[1].conduct_into(&[not_a[0], *b], &mut outputs[1..2])?;
        self.and_gates[2].conduct_into(&[*a, not_b[0]], &mut outputs[2..3])?;
        self.and_gates[3].conduct_into(&[*a, *b], &mut outputs[3..4])?;

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.not_a.transistor_count()
            + self.not_b.transistor_count()
            + self
                .and_gates
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
    }
}

/// Four-to-sixteen decoder: two 2-to-4 decoders and 16 AND gates.
#[derive(Debug, Default)]
pub struct Decoder4to16 {
    low_decoder: Decoder2to4,
    high_decoder: Decoder2to4,
    and_gates: [AndGate; 16],
}

impl Component for Decoder4to16 {
    const INPUTS: usize = 4;
    const OUTPUTS: usize = 16;

    /// Inputs:
    ///
    /// - 0..4: the value, least significant bit first.
    ///
    /// Outputs:
    ///
    /// - exactly one output is Driven(High): the one at the input value.
    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b, c, d] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 4,
                actual: inputs.len(),
            });
        };

        // `Decoder2to4(a, b)` activates index `2 * a + b`, so the most
        // significant bit of each pair must be given first.
        let mut high_group = [Signal::HighImpedance; 4];
        self.high_decoder
            .conduct_into(&[*d, *c], &mut high_group)?;
        let mut low_group = [Signal::HighImpedance; 4];
        self.low_decoder.conduct_into(&[*b, *a], &mut low_group)?;

        for index in 0..16 {
            self.and_gates[index]
                .conduct_into(&[high_group[index / 4], low_group[index % 4]], &mut outputs[index..index + 1])?;
        }

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.low_decoder.transistor_count()
            + self.high_decoder.transistor_count()
            + self
                .and_gates
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
    }
}

#[cfg(test)]
mod decoder2to4_tests {
    use super::*;
    use crate::hardware::Bit;

    #[test]
    fn test_truth_table() {
        let decoder = Decoder2to4::default();

        let expected = [
            [Bit::High, Bit::Low, Bit::Low, Bit::Low],
            [Bit::Low, Bit::High, Bit::Low, Bit::Low],
            [Bit::Low, Bit::Low, Bit::High, Bit::Low],
            [Bit::Low, Bit::Low, Bit::Low, Bit::High],
        ];

        for (a_index, a) in [Bit::Low, Bit::High].iter().enumerate() {
            for (b_index, b) in [Bit::Low, Bit::High].iter().enumerate() {
                assert_eq!(
                    decoder.compute(&[*a, *b]),
                    Ok(expected[2 * a_index + b_index].to_vec())
                );
            }
        }
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(Decoder2to4::default().transistor_count(), 28);
    }
}

#[cfg(test)]
mod decoder4to16_tests {
    use super::*;
    use crate::hardware::Bit;

    #[test]
    fn test_activates_exactly_one_line() {
        let decoder = Decoder4to16::default();

        for value in 0u8..16 {
            let bits: Vec<Bit> = (0..4)
                .map(|index| {
                    if (value >> index) & 1 == 1 {
                        Bit::High
                    } else {
                        Bit::Low
                    }
                })
                .collect();

            let outputs = decoder
                .compute(&bits)
                .expect("Decoder4to16 computation failed");

            assert_eq!(outputs[value as usize], Bit::High);
            assert_eq!(
                outputs
                    .iter()
                    .filter(|output| **output == Bit::High)
                    .count(),
                1
            );
        }
    }

    #[test]
    fn test_rejects_inputs_not_4_bits_long() {
        let decoder = Decoder4to16::default();

        assert_eq!(
            decoder.conduct(&[Signal::HighImpedance; 3]),
            Err(HardwareError::InvalidInputCount {
                expected: 4,
                actual: 3,
            })
        );

        assert_eq!(
            decoder.conduct(&[Signal::HighImpedance; 5]),
            Err(HardwareError::InvalidInputCount {
                expected: 4,
                actual: 5,
            })
        );
    }

    #[test]
    fn test_transistor_count() {
        assert_eq!(Decoder4to16::default().transistor_count(), 152);
    }
}
