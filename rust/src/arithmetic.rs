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
use crate::hardware::{Component, HardwareError, Nmos, Signal};

/// One-bit adder without carry input: sum = A XOR B, carry = A AND B.
#[derive(Debug, Default)]
pub struct HalfAdder {
    xor: XorGate,
    and: AndGate,
}

impl Component for HalfAdder {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let sum = self.xor.conduct(&[*a, *b])?;
        let carry = self.and.conduct(&[*a, *b])?;
        Ok(vec![sum[0], carry[0]])
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
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [a, b, carry_in] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 3,
                actual: inputs.len(),
            });
        };
        let result1 = self.half_adder1.conduct(&[*a, *b])?;
        let result2 = self.half_adder2.conduct(&[result1[0], *carry_in])?;
        let carry_out = self.or.conduct(&[result1[1], result2[1]])?;
        Ok(vec![result2[0], carry_out[0]])
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
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        if inputs.len() != 17 {
            return Err(HardwareError::InvalidInputCount {
                expected: 17,
                actual: inputs.len(),
            });
        }

        let mut carry = inputs[16];
        let mut result = Vec::with_capacity(9);

        for (full_adder, (a, b)) in self
            .full_adders
            .iter()
            .zip(inputs[..8].iter().zip(inputs[8..16].iter()))
        {
            let output = full_adder.conduct(&[*a, *b, carry])?;

            result.push(output[0]);
            carry = output[1];
        }

        result.push(carry);

        Ok(result)
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
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        if inputs.len() != 8 {
            return Err(HardwareError::InvalidInputCount {
                expected: 8,
                actual: inputs.len(),
            });
        }

        let pair_01 = self.or_gates[0].conduct(&[inputs[0], inputs[1]])?;
        let pair_23 = self.or_gates[1].conduct(&[inputs[2], inputs[3]])?;
        let pair_45 = self.or_gates[2].conduct(&[inputs[4], inputs[5]])?;
        let pair_67 = self.or_gates[3].conduct(&[inputs[6], inputs[7]])?;

        let half_0123 = self.or_gates[4].conduct(&[pair_01[0], pair_23[0]])?;
        let half_4567 = self.or_gates[5].conduct(&[pair_45[0], pair_67[0]])?;

        let any_bit_high = self.or_gates[6].conduct(&[half_0123[0], half_4567[0]])?;

        let zero = self.not.conduct(&[any_bit_high[0]])?;

        Ok(vec![zero[0]])
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
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        if inputs.len() != 18 {
            return Err(HardwareError::InvalidInputCount {
                expected: 18,
                actual: inputs.len(),
            });
        }

        let a = &inputs[0..8];
        let b = &inputs[8..16];
        let subtract = inputs[16];
        let load = inputs[17];

        // B' = B XOR subtract
        //
        // subtract = 0 -> B' = B
        // subtract = 1 -> B' = NOT B
        let mut modified_b = Vec::with_capacity(8);

        for (xor, b_bit) in self.b_xors.iter().zip(b.iter()) {
            let bit = xor.conduct(&[*b_bit, subtract])?;
            modified_b.push(bit[0]);
        }

        // A + B' + subtract
        //
        // Addition:
        //     A + B + 0
        //
        // Subtraction:
        //     A + NOT(B) + 1
        let mut adder_inputs = Vec::with_capacity(17);

        adder_inputs.extend_from_slice(a);
        adder_inputs.extend_from_slice(&modified_b);
        adder_inputs.push(subtract);

        let adder_output = self.adder.conduct(&adder_inputs)?;

        // Adder8Bits returns:
        //
        // [sum0, sum1, ..., sum7, carry]
        //
        let sum = &adder_output[0..8];
        let carry_flag = adder_output[8];

        // Zero flag is computed from the actual arithmetic result,
        // independently of whether the result is currently driving
        // the bus.
        let zero_flag = self.zero_detector.conduct(sum)?[0];

        // Tri-state output buffer.
        //
        // load = 1 -> result drives the bus
        // load = 0 -> result is high impedance
        let mut result = Vec::with_capacity(10);

        for (nmos, sum_bit) in self.output_nmos.iter().zip(sum.iter()) {
            let output = nmos.conduct(&[load, *sum_bit])?;
            result.push(output[0]);
        }

        result.push(carry_flag);
        result.push(zero_flag);

        Ok(result)
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
        use crate::hardware::{bits_to_int, int_to_bits};

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
