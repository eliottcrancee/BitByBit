//! Arithmetic circuits layered on top of [`crate::gates`].
//!
//! * [`HalfAdder`] — XOR (sum) + AND (carry), 22T.
//! * [`FullAdder`] — two half adders and an OR gate, 50T.
//! * [`Adder8Bits`] — eight chained full adders (ripple carry).
//! * [`ZeroDetector8Bits`] — OR tree followed by an inverter.
//! * [`ALU8Bits`] — an adder and a zero detector; `subtract` inverts
//!   the B operand and feeds `carry_in`, so SUB computes
//!   `A + NOT(B) + 1` (two's complement).

use crate::gates::{AndGate, NotGate, OrGate, XorGate};
use crate::hardware::{Bit, Component, HardwareError, Nmos, Signal};
use crate::FAST;

/// One-bit adder without carry input: sum = A XOR B, carry = A AND B.
#[derive(Debug, Default)]
pub struct HalfAdder {
    xor: XorGate,
    and: AndGate,
}

impl Component for HalfAdder {
    const INPUTS: usize = 2;
    const OUTPUTS: usize = 2;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        };

        let mut single = [Signal::HighImpedance; 1];

        self.xor.conduct_into(&[*a, *b], &mut single)?;
        outputs[0] = single[0];

        self.and.conduct_into(&[*a, *b], &mut single)?;
        outputs[1] = single[0];

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.xor.transistor_count() + self.and.transistor_count()
    }
}

/// Full adder: two chained half adders, carry = carry1 OR carry2.
#[derive(Debug, Default)]
pub struct FullAdder {
    half_adder1: HalfAdder,
    half_adder2: HalfAdder,
    or: OrGate,
}

impl Component for FullAdder {
    const INPUTS: usize = 3;
    const OUTPUTS: usize = 2;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b, carry_in] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        };

        let mut ha1_out = [Signal::HighImpedance; 2];
        self.half_adder1.conduct_into(&[*a, *b], &mut ha1_out)?;

        let mut ha2_out = [Signal::HighImpedance; 2];
        self.half_adder2.conduct_into(&[ha1_out[0], *carry_in], &mut ha2_out)?;

        let mut single = [Signal::HighImpedance; 1];
        self.or.conduct_into(&[ha1_out[1], ha2_out[1]], &mut single)?;

        outputs[0] = ha2_out[0];
        outputs[1] = single[0];

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.half_adder1.transistor_count()
            + self.half_adder2.transistor_count()
            + self.or.transistor_count()
    }
}

/// Eight-bit ripple-carry adder.
#[derive(Debug, Default)]
pub struct Adder8Bits {
    full_adders: [FullAdder; 8],
}

impl Component for Adder8Bits {
    const INPUTS: usize = 17;
    const OUTPUTS: usize = 9;

    /// Inputs:
    ///
    /// - 0..8:   a, least significant bit first
    /// - 8..16:  b, least significant bit first
    /// - 16:     carry_in
    ///
    /// Outputs:
    ///
    /// - 0..8: sum, least significant bit first
    /// - 8:    carry_out
    ///
    /// Computes `a + b + carry_in`.
    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let mut carry = inputs[16];
        let mut fa_out = [Signal::HighImpedance; 2];

        for (index, (full_adder, (a, b))) in self
            .full_adders
            .iter()
            .zip(inputs[..8].iter().zip(inputs[8..16].iter()))
            .enumerate()
        {
            full_adder.conduct_into(&[*a, *b, carry], &mut fa_out)?;

            outputs[index] = fa_out[0];
            carry = fa_out[1];
        }

        outputs[8] = carry;

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.full_adders
            .iter()
            .map(|fa| fa.transistor_count())
            .sum()
    }
}

/// Detects the all-zero byte: an OR tree over the 7 non-redundant
/// pairs of bits, inverted once.
#[derive(Debug, Default)]
pub struct ZeroDetector8Bits {
    or_gates: [OrGate; 7],
    not: NotGate,
}

impl Component for ZeroDetector8Bits {
    const INPUTS: usize = 8;
    const OUTPUTS: usize = 1;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let mut single = [Signal::HighImpedance; 1];

        self.or_gates[0].conduct_into(&[inputs[0], inputs[1]], &mut single)?;
        let pair_01 = single[0];
        self.or_gates[1].conduct_into(&[inputs[2], inputs[3]], &mut single)?;
        let pair_23 = single[0];
        self.or_gates[2].conduct_into(&[inputs[4], inputs[5]], &mut single)?;
        let pair_45 = single[0];
        self.or_gates[3].conduct_into(&[inputs[6], inputs[7]], &mut single)?;
        let pair_67 = single[0];

        self.or_gates[4].conduct_into(&[pair_01, pair_23], &mut single)?;
        let half_0123 = single[0];
        self.or_gates[5].conduct_into(&[pair_45, pair_67], &mut single)?;
        let half_4567 = single[0];

        self.or_gates[6].conduct_into(&[half_0123, half_4567], &mut single)?;

        self.not.conduct_into(&[single[0]], outputs)
    }

    fn transistor_count(&self) -> usize {
        self.or_gates
            .iter()
            .map(|gate| gate.transistor_count())
            .sum::<usize>()
            + self.not.transistor_count()
    }
}

/// The SAP-1 arithmetic and logic unit.
///
/// Inputs: `a`, `b`, `subtract` and `load`. When `subtract` is HIGH the
/// B operand is inverted through NOT gates and `carry_in` is forced to
/// HIGH through an NMOS, computing the two's complement difference
/// `A + NOT(B) + 1`; otherwise the sum `A + B` is computed. The output
/// bus drives only when `load` is HIGH, the flags are always exposed.
#[derive(Debug, Default)]
pub struct ALU8Bits {
    adder: Adder8Bits,
    b_xors: [XorGate; 8],
    zero_detector: ZeroDetector8Bits,
    output_nmos: [Nmos; 8],
}

impl Component for ALU8Bits {
    /// Inputs:
    ///
    /// - 0..8   : A, least-significant bit first
    /// - 8..16  : B, least-significant bit first
    /// - 16     : subtract
    /// - 17     : load
    ///
    /// Outputs:
    ///
    /// - 0..8   : result, least-significant bit first
    /// - 8      : carry flag
    /// - 9      : zero flag
    ///
    /// When `subtract = Low`:
    ///
    ///     result = A + B
    ///
    /// When `subtract = High`:
    ///
    ///     result = A + NOT(B) + 1
    ///
    /// Therefore the carry flag follows the two's-complement
    /// subtraction convention:
    ///
    ///     carry = 1 <=> A >= B
    ///
    /// The result is gated by `load`. When `load = Low`, the
    /// result bits are HighImpedance, while the flags remain
    /// valid.
    ///
    /// With `FAST`, a Low `load` skips the whole computation: the
    /// result bits *and* the flags come out HighImpedance, since a
    /// caller reading the flags with `load` Low would capture a
    /// floating line anyway.
    const INPUTS: usize = 18;
    const OUTPUTS: usize = 10;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let a = &inputs[0..8];
        let b = &inputs[8..16];
        let subtract = inputs[16];
        let load = inputs[17];

        // With `FAST`, an ALU whose output buffer is disabled cannot
        // drive anything: skip the XOR row, the adder and the
        // zero detector entirely.
        if FAST && load == Signal::Driven(Bit::Low) {
            outputs.fill(Signal::HighImpedance);
            return Ok(());
        }

        // B' = B XOR subtract
        //
        // subtract = 0 -> B' = B
        // subtract = 1 -> B' = NOT B
        let mut modified_b = [Signal::HighImpedance; 8];

        for index in 0..8 {
            self.b_xors[index].conduct_into(&[b[index], subtract], &mut modified_b[index..index + 1])?;
        }

        // A + B' + subtract
        //
        // Addition:
        //     A + B + 0
        //
        // Subtraction:
        //     A + NOT(B) + 1
        let mut adder_inputs = [Signal::HighImpedance; 17];
        adder_inputs[..8].copy_from_slice(a);
        adder_inputs[8..16].copy_from_slice(&modified_b);
        adder_inputs[16] = subtract;

        // Adder8Bits returns [sum0, ..., sum7, carry].
        let mut adder_output = [Signal::HighImpedance; 9];
        self.adder.conduct_into(&adder_inputs, &mut adder_output)?;

        // Zero flag is computed from the actual arithmetic result,
        // independently of whether the result is currently driving
        // the bus.
        let mut zero_flag = [Signal::HighImpedance; 1];
        self.zero_detector
            .conduct_into(&adder_output[0..8], &mut zero_flag)?;

        // Tri-state output buffer.
        //
        // load = 1 -> result drives the bus
        // load = 0 -> result is high impedance
        for index in 0..8 {
            self.output_nmos[index].conduct_into(
                &[load, adder_output[index]],
                &mut outputs[index..index + 1],
            )?;
        }

        outputs[8] = adder_output[8];
        outputs[9] = zero_flag[0];

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.adder.transistor_count()
            + self
                .b_xors
                .iter()
                .map(|xor| xor.transistor_count())
                .sum::<usize>()
            + self.zero_detector.transistor_count()
            + self
                .output_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hardware::Bit;

    #[test]
    fn test_half_adder() {
        let adder = HalfAdder::default();
        assert_eq!(
            adder.compute(&[Bit::Low, Bit::Low]),
            Ok(vec![Bit::Low, Bit::Low])
        );
        assert_eq!(
            adder.compute(&[Bit::Low, Bit::High]),
            Ok(vec![Bit::High, Bit::Low])
        );
        assert_eq!(
            adder.compute(&[Bit::High, Bit::Low]),
            Ok(vec![Bit::High, Bit::Low])
        );
        assert_eq!(
            adder.compute(&[Bit::High, Bit::High]),
            Ok(vec![Bit::Low, Bit::High])
        );
        assert_eq!(adder.transistor_count(), 22);
    }

    #[test]
    fn test_full_adder() {
        let adder = FullAdder::default();
        assert_eq!(
            adder.compute(&[Bit::Low, Bit::Low, Bit::Low]),
            Ok(vec![Bit::Low, Bit::Low])
        );
        assert_eq!(
            adder.compute(&[Bit::Low, Bit::Low, Bit::High]),
            Ok(vec![Bit::High, Bit::Low])
        );
        assert_eq!(
            adder.compute(&[Bit::Low, Bit::High, Bit::Low]),
            Ok(vec![Bit::High, Bit::Low])
        );
        assert_eq!(
            adder.compute(&[Bit::Low, Bit::High, Bit::High]),
            Ok(vec![Bit::Low, Bit::High])
        );
        assert_eq!(
            adder.compute(&[Bit::High, Bit::Low, Bit::Low]),
            Ok(vec![Bit::High, Bit::Low])
        );
        assert_eq!(
            adder.compute(&[Bit::High, Bit::Low, Bit::High]),
            Ok(vec![Bit::Low, Bit::High])
        );
        assert_eq!(
            adder.compute(&[Bit::High, Bit::High, Bit::Low]),
            Ok(vec![Bit::Low, Bit::High])
        );
        assert_eq!(
            adder.compute(&[Bit::High, Bit::High, Bit::High]),
            Ok(vec![Bit::High, Bit::High])
        );
        assert_eq!(adder.transistor_count(), 50);
    }

    #[test]
    fn test_adder_8_bits() {
        let adder = Adder8Bits::default();
        let a = [
            Bit::High,
            Bit::Low,
            Bit::High,
            Bit::Low,
            Bit::High,
            Bit::Low,
            Bit::High,
            Bit::Low,
        ];
        let b = [
            Bit::Low,
            Bit::High,
            Bit::Low,
            Bit::High,
            Bit::Low,
            Bit::High,
            Bit::Low,
            Bit::High,
        ];
        let inputs: Vec<Bit> = a
            .iter()
            .chain(b.iter())
            .cloned()
            .chain([Bit::Low])
            .collect();
        let result = adder
            .compute(&inputs)
            .expect("Adder8Bits computation failed");
        assert_eq!(
            result,
            vec![
                Bit::High,
                Bit::High,
                Bit::High,
                Bit::High,
                Bit::High,
                Bit::High,
                Bit::High,
                Bit::High,
                Bit::Low,
            ]
        );
    }

    #[test]
    fn test_adder_8_bits_with_carry_in() {
        let adder = Adder8Bits::default();

        // 0 + 0 + 1 = 1, no carry out
        let inputs = [Bit::Low; 16]
            .iter()
            .cloned()
            .chain([Bit::High])
            .collect::<Vec<Bit>>();
        let mut expected = vec![Bit::Low; 9];
        expected[0] = Bit::High;
        assert_eq!(adder.compute(&inputs), Ok(expected));

        // 255 + 0 + 1 = 256 -> 0 with carry out
        let inputs = [Bit::High; 8]
            .iter()
            .cloned()
            .chain([Bit::Low; 8])
            .chain([Bit::High])
            .collect::<Vec<Bit>>();
        let result = adder
            .compute(&inputs)
            .expect("Adder8Bits computation failed");
        assert_eq!(result[0..8], [Bit::Low; 8]);
        assert_eq!(result[8], Bit::High);
    }

    #[test]
    fn test_alu8bits_subtraction() {
        use crate::utils::{bits_to_int, int_to_bits};

        let alu = ALU8Bits::default();

        // 9 - 4 = 5, carry High (no borrow: 9 >= 4), zero Low.
        let subtract_case = |a_value: u8, b_value: u8| -> Vec<Bit> {
            let mut inputs: Vec<Bit> = Vec::with_capacity(18);
            inputs.extend(int_to_bits(a_value));
            inputs.extend(int_to_bits(b_value));
            inputs.extend([Bit::High, Bit::High]); // subtract, load
            alu.compute(&inputs).expect("ALU computation failed")
        };

        let result = subtract_case(9, 4);
        assert_eq!(bits_to_int(&result[0..8], false), 5);
        assert_eq!(result[8], Bit::High); // no borrow
        assert_eq!(result[9], Bit::Low); // result != 0

        // 3 - 3 = 0, carry High, zero High.
        let result = subtract_case(3, 3);
        assert_eq!(bits_to_int(&result[0..8], false), 0);
        assert_eq!(result[8], Bit::High);
        assert_eq!(result[9], Bit::High);

        // 3 - 4 = 255 (including borrow), carry Low, zero Low.
        let result = subtract_case(3, 4);
        assert_eq!(bits_to_int(&result[0..8], false), 255);
        assert_eq!(result[8], Bit::Low);
        assert_eq!(result[9], Bit::Low);
    }
}
