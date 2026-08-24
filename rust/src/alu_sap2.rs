//! The SAP-2 arithmetic and logic unit, layered on top of
//! [`crate::arithmetic`] and [`crate::gates`].
//!
//! * [`Alu8BitsSap2`] — a full ALU supporting addition, subtraction
//!   and the bitwise logic operations AND, OR, XOR and NOT, selected
//!   by a 3-bit operation code. Every candidate result drives the
//!   output bus through its own tri-state transistor, exactly like a
//!   real gate-level ALU.

use crate::arithmetic::{Adder8Bits, ZeroDetector8Bits};
use crate::gates::{AndGate, NotGate, OrGate, XorGate};
use crate::hardware::{
    bits_to_int, bus8, int_to_bits, Bit, Component, HardwareError, Nmos, Signal,
};

/// Operation codes accepted by [`Alu8BitsSap2`], encoded on the three
/// `op` lines (`op2 op1 op0`, most significant bit first).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AluOperation {
    /// `000` — `A + B`
    Add,
    /// `001` — `A - B` (two's complement: `A + NOT(B) + 1`)
    Sub,
    /// `010` — `A AND B`
    And,
    /// `011` — `A OR B`
    Or,
    /// `100` — `A XOR B`
    Xor,
    /// `101` — `NOT A`
    Not,
}

impl AluOperation {
    /// The 3-bit encoding, most significant bit first.
    #[must_use]
    pub fn to_bits(self) -> [Bit; 3] {
        match self {
            Self::Add => [Bit::Low, Bit::Low, Bit::Low],
            Self::Sub => [Bit::Low, Bit::Low, Bit::High],
            Self::And => [Bit::Low, Bit::High, Bit::Low],
            Self::Or => [Bit::Low, Bit::High, Bit::High],
            Self::Xor => [Bit::High, Bit::Low, Bit::Low],
            Self::Not => [Bit::High, Bit::Low, Bit::High],
        }
    }
}

/// The SAP-2 arithmetic and logic unit.
///
/// Inputs:
///
/// - 0..8   : A, least-significant bit first
/// - 8..16  : B, least-significant bit first
/// - 16     : op2 (most significant bit of the operation code)
/// - 17     : op1
/// - 18     : op0 (least significant bit of the operation code)
/// - 19     : load
///
/// Outputs:
///
/// - 0..8   : result, least-significant bit first
/// - 8      : carry flag (driven only during ADD / SUB)
/// - 9      : zero flag
///
/// The operation decoder is pure hardware: the three op lines feed
/// three inverters whose outputs are physical nets, and each of the
/// six operation-enable nets is an AND chain wired to either the
/// direct line or its inverted copy — exactly like a real gate-level
/// decoder. No software branching anywhere.
#[derive(Debug, Default)]
pub struct Alu8BitsSap2 {
    // Shared arithmetic path.
    adder: Adder8Bits,
    b_xors: [XorGate; 8],

    // Operation decoder: one inverter per op line, then two AND gates
    // per enable net (6 enables x 2 = 12).
    not_op0: NotGate,
    not_op1: NotGate,
    not_op2: NotGate,
    decode_ands: [AndGate; 12],

    // Per-bit logic units.
    and_gates: [AndGate; 8],
    or_gates: [OrGate; 8],
    xor_gates: [XorGate; 8],
    not_gates: [NotGate; 8],

    // Tri-state drivers: six function slots per bit (ADD and SUB both
    // drive the adder sum), one for the carry and eight for the `load`
    // output buffer.
    output_nmos: [[Nmos; 6]; 8],
    carry_nmos: Nmos,
    load_buffer_nmos: [Nmos; 8],

    // Shared flag path (computed on the resolved internal bus).
    zero_detector: ZeroDetector8Bits,

    // Carry is exposed only when an arithmetic function is selected.
    arith_or: OrGate,
}

impl Alu8BitsSap2 {
    /// Convenience wrapper running one operation on integer operands
    /// and returning `(result, carry, zero)`.
    ///
    /// # Errors
    ///
    /// Propagates any [`HardwareError`] raised while computing.
    pub fn evaluate(
        &self,
        a: u8,
        b: u8,
        operation: AluOperation,
        load: bool,
    ) -> Result<(u8, bool, bool), HardwareError> {
        let mut inputs: Vec<Signal> = Vec::with_capacity(20);
        inputs.extend(int_to_bits(a).map(Signal::from));
        inputs.extend(int_to_bits(b).map(Signal::from));
        inputs.extend(operation.to_bits().map(Signal::from));
        inputs.push(Signal::from(if load { Bit::High } else { Bit::Low }));

        let output = self.conduct(&inputs)?;
        Ok((
            bits_to_int(
                &output[0..8]
                    .iter()
                    .map(|signal| match signal {
                        Signal::Driven(bit) => *bit,
                        Signal::HighImpedance => Bit::Low,
                    })
                    .collect::<Vec<Bit>>(),
                false,
            ),
            output[8] == Signal::Driven(Bit::High),
            output[9] == Signal::Driven(Bit::High),
        ))
    }
}

impl Component for Alu8BitsSap2 {
    /// See the type documentation for the input/output layout and the
    /// operation table.
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        if inputs.len() != 20 {
            return Err(HardwareError::InvalidInputCount {
                expected: 20,
                actual: inputs.len(),
            });
        }

        let a = &inputs[0..8];
        let b = &inputs[8..16];
        let op_msb = inputs[16];
        let op_mid = inputs[17];
        let op_lsb = inputs[18];
        let load = inputs[19];

        // ---- Operation decoder (pure gate wiring) ------------------
        //
        // The three inverters produce the complementary nets; each of
        // the six enable nets is an AND chain wired to either a direct
        // line or its complement:
        //
        //   ADD : NOT(op2) . NOT(op1) . NOT(op0)
        //   SUB : NOT(op2) . NOT(op1) . op0
        //   AND : NOT(op2) . op1      . NOT(op0)
        //   OR  : NOT(op2) . op1      . op0
        //   XOR : op2      . NOT(op1) . NOT(op0)
        //   NOT : op2      . NOT(op1) . op0
        let n_msb = self.not_op2.conduct(&[op_msb])?[0];
        let n_mid = self.not_op1.conduct(&[op_mid])?[0];
        let n_lsb = self.not_op0.conduct(&[op_lsb])?[0];

        let pair_add = self.decode_ands[0].conduct(&[n_msb, n_mid])?;
        let enable_add = self.decode_ands[1].conduct(&[pair_add[0], n_lsb])?[0];

        let pair_sub = self.decode_ands[2].conduct(&[n_msb, n_mid])?;
        let enable_sub = self.decode_ands[3].conduct(&[pair_sub[0], op_lsb])?[0];

        let pair_and = self.decode_ands[4].conduct(&[n_msb, op_mid])?;
        let enable_and = self.decode_ands[5].conduct(&[pair_and[0], n_lsb])?[0];

        let pair_or = self.decode_ands[6].conduct(&[n_msb, op_mid])?;
        let enable_or = self.decode_ands[7].conduct(&[pair_or[0], op_lsb])?[0];

        let pair_xor = self.decode_ands[8].conduct(&[op_msb, n_mid])?;
        let enable_xor = self.decode_ands[9].conduct(&[pair_xor[0], n_lsb])?[0];

        let pair_not = self.decode_ands[10].conduct(&[op_msb, n_mid])?;
        let enable_not = self.decode_ands[11].conduct(&[pair_not[0], op_lsb])?[0];

        let enables: [Signal; 6] = [
            enable_add,
            enable_sub,
            enable_and,
            enable_or,
            enable_xor,
            enable_not,
        ];

        // ---- Arithmetic path ---------------------------------------
        //
        // B' = B XOR sub, then A + B' + sub:
        //
        //   ADD -> A + B  + 0
        //   SUB -> A + NOT(B) + 1
        let mut modified_b = Vec::with_capacity(8);

        for (xor, b_bit) in self.b_xors.iter().zip(b.iter()) {
            modified_b.push(xor.conduct(&[*b_bit, enable_sub])?[0]);
        }

        let mut adder_inputs = Vec::with_capacity(17);
        adder_inputs.extend_from_slice(a);
        adder_inputs.extend_from_slice(&modified_b);
        adder_inputs.push(enable_sub);

        let adder_output = self.adder.conduct(&adder_inputs)?;
        let sum = &adder_output[0..8];
        let raw_carry = adder_output[8];

        // ---- Logic path: one candidate bit per function slot ---------
        //
        // Slots 0 and 1 both expose the adder sum (ADD and SUB select
        // the same value; only their tri-state enables differ), then
        // AND, OR, XOR and NOT follow.
        let mut candidates = [[Signal::HighImpedance; 6]; 8];

        for index in 0..8 {
            candidates[index][0] = sum[index];
            candidates[index][1] = sum[index];
            candidates[index][2] =
                self.and_gates[index].conduct(&[a[index], b[index]])?[0];
            candidates[index][3] =
                self.or_gates[index].conduct(&[a[index], b[index]])?[0];
            candidates[index][4] =
                self.xor_gates[index].conduct(&[a[index], b[index]])?[0];
            candidates[index][5] = self.not_gates[index].conduct(&[a[index]])?[0];
        }

        // ---- Tri-state bus: exactly one driver per bit --------------
        let mut drivers: Vec<[Signal; 8]> = Vec::with_capacity(6);

        for (function, enable) in enables.iter().enumerate() {
            let mut row = [Signal::HighImpedance; 8];

            for (index, slot) in row.iter_mut().enumerate() {
                *slot = self.output_nmos[index][function]
                    .conduct(&[*enable, candidates[index][function]])?[0];
            }

            drivers.push(row);
        }

        let resolved = bus8(drivers)?;

        // ---- Flags: always valid, independently of `load` -----------
        //
        // The carry only drives during arithmetic operations; it floats
        // otherwise. The zero flag is computed on the resolved internal
        // result.
        let zero_flag = self.zero_detector.conduct(resolved.as_slice())?[0];
        let carry_enable = self.arith_or.conduct(&[enable_add, enable_sub])?[0];
        let carry_flag = self.carry_nmos.conduct(&[carry_enable, raw_carry])?[0];

        // ---- Output buffer gated by `load` --------------------------
        let mut result: Vec<Signal> = Vec::with_capacity(10);

        for (nmos, bit) in self.load_buffer_nmos.iter().zip(resolved.iter()) {
            result.push(nmos.conduct(&[load, *bit])?[0]);
        }

        result.push(carry_flag);
        result.push(zero_flag);

        Ok(result)
    }

    fn transistor_count(&self) -> usize {
        macro_rules! sum_all {
            ($field:expr) => {
                $field.iter().map(|g| g.transistor_count()).sum::<usize>()
            };
        }

        self.adder.transistor_count()
            + sum_all!(self.b_xors)
            + self.not_op0.transistor_count()
            + self.not_op1.transistor_count()
            + self.not_op2.transistor_count()
            + sum_all!(self.decode_ands)
            + sum_all!(self.and_gates)
            + sum_all!(self.or_gates)
            + sum_all!(self.xor_gates)
            + sum_all!(self.not_gates)
            + self
                .output_nmos
                .iter()
                .flat_map(|row| row.iter())
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
            + self.carry_nmos.transistor_count()
            + sum_all!(self.load_buffer_nmos)
            + self.arith_or.transistor_count()
            + self.zero_detector.transistor_count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use AluOperation::*;

    #[test]
    fn test_operation_encodings() {
        assert_eq!(Add.to_bits(), [Bit::Low, Bit::Low, Bit::Low]);
        assert_eq!(Sub.to_bits(), [Bit::Low, Bit::Low, Bit::High]);
        assert_eq!(And.to_bits(), [Bit::Low, Bit::High, Bit::Low]);
        assert_eq!(Or.to_bits(), [Bit::Low, Bit::High, Bit::High]);
        assert_eq!(Xor.to_bits(), [Bit::High, Bit::Low, Bit::Low]);
        assert_eq!(Not.to_bits(), [Bit::High, Bit::Low, Bit::High]);
    }

    #[test]
    fn test_addition() {
        let alu = Alu8BitsSap2::default();

        // 9 + 4 = 13, no carry.
        assert_eq!(alu.evaluate(9, 4, Add, true).unwrap(), (13, false, false));

        // 200 + 100 = 300 -> 44 with carry out.
        assert_eq!(
            alu.evaluate(200, 100, Add, true).unwrap(),
            (44, true, false)
        );

        // 0 + 0 = 0 sets the zero flag.
        assert_eq!(alu.evaluate(0, 0, Add, true).unwrap(), (0, false, true));
    }

    #[test]
    fn test_subtraction() {
        let alu = Alu8BitsSap2::default();

        // 9 - 4 = 5, carry High (no borrow).
        assert_eq!(alu.evaluate(9, 4, Sub, true).unwrap(), (5, true, false));

        // 3 - 3 = 0, carry High, zero flag set.
        assert_eq!(alu.evaluate(3, 3, Sub, true).unwrap(), (0, true, true));

        // 3 - 4 wraps to 255, carry Low (borrow).
        assert_eq!(
            alu.evaluate(3, 4, Sub, true).unwrap(),
            (255, false, false)
        );
    }

    #[test]
    fn test_logic_operations() {
        let alu = Alu8BitsSap2::default();

        let (result, carry, _) =
            alu.evaluate(0b1100_1010, 0b1010_1100, And, true).unwrap();
        assert_eq!(result, 0b1000_1000);
        assert!(!carry); // logic ops do not drive the carry

        let (result, _, _) =
            alu.evaluate(0b1100_1010, 0b1010_1100, Or, true).unwrap();
        assert_eq!(result, 0b1110_1110);

        let (result, _, _) =
            alu.evaluate(0b1100_1010, 0b1010_1100, Xor, true).unwrap();
        assert_eq!(result, 0b0110_0110);

        let (result, _, _) = alu.evaluate(0b1100_1010, 0xFF, Not, true).unwrap();
        assert_eq!(result, 0b0011_0101);
    }

    #[test]
    fn test_load_gates_the_result_but_not_flags() {
        let alu = Alu8BitsSap2::default();

        let mut inputs: Vec<Signal> = Vec::with_capacity(20);
        inputs.extend(int_to_bits(9).map(Signal::Driven));
        inputs.extend(int_to_bits(4).map(Signal::Driven));
        inputs.extend(Add.to_bits().map(Signal::Driven));
        inputs.push(Signal::HighImpedance); // load floating

        let output = alu.conduct(&inputs).unwrap();

        // Result floats, flags stay valid.
        for bit in &output[0..8] {
            assert_eq!(*bit, Signal::HighImpedance);
        }
        assert_eq!(output[8], Signal::Driven(Bit::Low)); // no carry
        assert_eq!(output[9], Signal::Driven(Bit::Low)); // result != 0
    }

    #[test]
    fn test_rejects_wrong_input_count() {
        let alu = Alu8BitsSap2::default();
        assert_eq!(
            alu.conduct(&[Signal::HighImpedance; 19]),
            Err(HardwareError::InvalidInputCount {
                expected: 20,
                actual: 19,
            })
        );
    }

    #[test]
    fn test_exhaustive_against_reference() {
        let alu = Alu8BitsSap2::default();

        // Stratified sample for the four logic operations: the
        // stepping is a full-period LCG over u8, so ~30 values of each
        // operand cover all bit patterns of the low nibble and a good
        // spread of the high one.
        for step in 0..30_u8 {
            let a_value = step.wrapping_mul(29).wrapping_add(17);
            let b_value = step.wrapping_mul(61).wrapping_add(5);

            assert_eq!(
                alu.evaluate(a_value, b_value, And, true).unwrap().0,
                a_value & b_value,
                "AND failed for {a_value}, {b_value}"
            );
            assert_eq!(
                alu.evaluate(a_value, b_value, Or, true).unwrap().0,
                a_value | b_value,
                "OR failed for {a_value}, {b_value}"
            );
            assert_eq!(
                alu.evaluate(a_value, b_value, Xor, true).unwrap().0,
                a_value ^ b_value,
                "XOR failed for {a_value}, {b_value}"
            );
            assert_eq!(
                alu.evaluate(a_value, b_value, Not, true).unwrap().0,
                !a_value,
                "NOT failed for {a_value}"
            );
        }

        // All ADD / SUB pairs against the reference semantics.
        for delta in 0..=255_u8 {
            let (result, carry, _) = alu.evaluate(delta, 137, Add, true).unwrap();
            let total = u16::from(delta) + 137_u16;
            assert_eq!(result, (total & 0xFF) as u8);
            assert_eq!(carry, total > 0xFF);

            let (result, carry, _) = alu.evaluate(delta, 42, Sub, true).unwrap();
            assert_eq!(result, delta.wrapping_sub(42));
            assert_eq!(carry, delta >= 42);
        }
    }
}





