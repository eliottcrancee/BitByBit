//! Arithmetic circuits layered on top of [`crate::gates`].
//!
//! * [`HalfAdder`] — XOR (sum) + AND (carry), 22T.
//! * [`FullAdder`] — two half adders and an OR gate, 50T.
//! * [`Adder8Bits`] — eight chained full adders (ripple carry).
//! * [`ZeroDetector8Bits`] — OR tree followed by an inverter.
//! * [`AluSap1`] — the SAP-1 ALU: an adder and a zero detector;
//!   `subtract` inverts the B operand and feeds `carry_in`, so SUB
//!   computes `A + NOT(B) + 1` (two's complement).
//! * [`AluSap2`] — the SAP-2 ALU: addition, subtraction and the
//!   bitwise logic operations AND, OR, XOR and NOT, selected by a
//!   3-bit operation code. Every candidate result drives the output
//!   bus through its own tri-state transistor, exactly like a real
//!   gate-level ALU.

use crate::gates::{AndGate, NotGate, OrGate, XorGate};
use crate::hardware::{bus8, wire, Bit, Component, HardwareError, Nmos, Signal};
use crate::mux::Mux2x1;
use crate::utils::{bits_to_int, int_to_bits};
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

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
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

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        let [a, b, carry_in] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        };

        let mut ha1_out = [Signal::HighImpedance; 2];
        self.half_adder1.conduct_into(&[*a, *b], &mut ha1_out)?;

        let mut ha2_out = [Signal::HighImpedance; 2];
        self.half_adder2
            .conduct_into(&[ha1_out[0], *carry_in], &mut ha2_out)?;

        let mut single = [Signal::HighImpedance; 1];
        self.or
            .conduct_into(&[ha1_out[1], ha2_out[1]], &mut single)?;

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
    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
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

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
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

/// Sign flag detector: buffers bit 7 of an 8-bit two's-complement
/// value — the sign bit — through a single NMOS whose gate is tied
/// HIGH, so the output always mirrors `inputs[7]`.
#[derive(Debug, Default)]
pub struct SignDetector8Bits {
    buffer: Nmos,
}

impl Component for SignDetector8Bits {
    /// Inputs:
    ///
    /// - 0..8: the value, least-significant bit first
    ///
    /// Outputs:
    ///
    /// - 0: `inputs[7]` (the sign bit), always driven.
    const INPUTS: usize = 8;
    const OUTPUTS: usize = 1;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let tied_high = [Signal::Driven(Bit::High)];
        self.buffer
            .conduct_into(&[tied_high[0], inputs[7]], &mut outputs[0..1])
    }

    fn transistor_count(&self) -> usize {
        self.buffer.transistor_count()
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
pub struct AluSap1 {
    adder: Adder8Bits,
    b_xors: [XorGate; 8],
    zero_detector: ZeroDetector8Bits,
    output_nmos: [Nmos; 8],
}

impl Component for AluSap1 {
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

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
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
            self.b_xors[index]
                .conduct_into(&[b[index], subtract], &mut modified_b[index..index + 1])?;
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
            self.output_nmos[index]
                .conduct_into(&[load, adder_output[index]], &mut outputs[index..index + 1])?;
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

/// Operation codes accepted by [`AluSap2`], encoded on the three
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
    /// `110` — `A + 1`
    Inc,
    /// `111` — `A - 1`
    Dec,
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
            Self::Inc => [Bit::High, Bit::High, Bit::Low],
            Self::Dec => [Bit::High, Bit::High, Bit::High],
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
pub struct AluSap2 {
    // Shared arithmetic path.
    adder: Adder8Bits,
    b_xors: [XorGate; 8],

    // Operation decoder: one inverter per op line, then two AND gates
    // per enable net (8 enables x 2, minus 1 shared upper term for
    // INC/DEC = 15).
    not_op0: NotGate,
    not_op1: NotGate,
    not_op2: NotGate,
    decode_ands: [AndGate; 15],

    // Increment/decrement force the adder's B input to `00000001` /
    // `11111111`: one mux level per force, per bit.
    inc_muxes: [Mux2x1; 8],
    dec_muxes: [Mux2x1; 8],

    // Per-bit logic units.
    and_gates: [AndGate; 8],
    or_gates: [OrGate; 8],
    xor_gates: [XorGate; 8],
    not_gates: [NotGate; 8],

    // Tri-state drivers: eight function slots per bit (ADD, SUB, INC
    // and DEC all drive the adder sum), one for the carry and eight
    // for the `load` output buffer.
    output_nmos: [[Nmos; 8]; 8],
    carry_nmos: Nmos,
    load_buffer_nmos: [Nmos; 8],

    // Shared flag path (computed on the resolved internal bus).
    zero_detector: ZeroDetector8Bits,

    // Carry is exposed only when an arithmetic function (including
    // INC/DEC) is selected, so the flag capture never sees a float.
    arith_or: OrGate,
    arith_or_inc_dec: OrGate,
    arith_or_final: OrGate,
    // Carry pull-down for the logic operations (AND/OR/XOR/NOT):
    // the 8080 clears CY there, so a grounded NMOS driven by the OR
    // of the four logic enables keeps the flag valid instead of
    // floating it.
    logic_or_low: OrGate,
    logic_or_high: OrGate,
    logic_or: OrGate,
    carry_pulldown: Nmos,
}

impl AluSap2 {
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
        let mut inputs = [Signal::HighImpedance; AluSap2::INPUTS];
        for (index, bit) in int_to_bits(a).into_iter().enumerate() {
            inputs[index] = Signal::from(bit);
        }

        for (index, bit) in int_to_bits(b).into_iter().enumerate() {
            inputs[8 + index] = Signal::from(bit);
        }
        for (index, bit) in operation.to_bits().into_iter().enumerate() {
            inputs[16 + index] = Signal::from(bit);
        }
        inputs[19] = Signal::from(if load { Bit::High } else { Bit::Low });

        let output = self.conduct(&inputs)?;
        let mut result_bits = [Bit::Low; 8];

        for (index, signal) in output[0..8].iter().enumerate() {
            result_bits[index] = match signal {
                Signal::Driven(bit) => *bit,
                Signal::HighImpedance => Bit::Low,
            };
        }

        Ok((
            bits_to_int(&result_bits, false),
            output[8] == Signal::Driven(Bit::High),
            output[9] == Signal::Driven(Bit::High),
        ))
    }
}

impl Component for AluSap2 {
    /// See the type documentation for the input/output layout and the
    /// operation table.
    const INPUTS: usize = 20;
    const OUTPUTS: usize = 10;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
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
        //   INC : op2      . op1      . NOT(op0)
        //   DEC : op2      . op1      . op0
        let mut single = [Signal::HighImpedance; 1];

        self.not_op2.conduct_into(&[op_msb], &mut single)?;
        let n_msb = single[0];
        self.not_op1.conduct_into(&[op_mid], &mut single)?;
        let n_mid = single[0];
        self.not_op0.conduct_into(&[op_lsb], &mut single)?;
        let n_lsb = single[0];

        self.decode_ands[0].conduct_into(&[n_msb, n_mid], &mut single)?;
        let upper_add = single[0];
        self.decode_ands[1].conduct_into(&[upper_add, n_lsb], &mut single)?;
        let enable_add = single[0];

        self.decode_ands[2].conduct_into(&[n_msb, n_mid], &mut single)?;
        let upper_sub = single[0];
        self.decode_ands[3].conduct_into(&[upper_sub, op_lsb], &mut single)?;
        let enable_sub = single[0];

        self.decode_ands[4].conduct_into(&[n_msb, op_mid], &mut single)?;
        let upper_and = single[0];
        self.decode_ands[5].conduct_into(&[upper_and, n_lsb], &mut single)?;
        let enable_and = single[0];

        self.decode_ands[6].conduct_into(&[n_msb, op_mid], &mut single)?;
        let upper_or = single[0];
        self.decode_ands[7].conduct_into(&[upper_or, op_lsb], &mut single)?;
        let enable_or = single[0];

        self.decode_ands[8].conduct_into(&[op_msb, n_mid], &mut single)?;
        let upper_xor = single[0];
        self.decode_ands[9].conduct_into(&[upper_xor, n_lsb], &mut single)?;
        let enable_xor = single[0];

        self.decode_ands[10].conduct_into(&[op_msb, n_mid], &mut single)?;
        let upper_not = single[0];
        self.decode_ands[11].conduct_into(&[upper_not, op_lsb], &mut single)?;
        let enable_not = single[0];

        // INC and DEC share the `op2 . op1` upper term.
        self.decode_ands[12].conduct_into(&[op_msb, op_mid], &mut single)?;
        let upper_incdec = single[0];
        self.decode_ands[13].conduct_into(&[upper_incdec, n_lsb], &mut single)?;
        let enable_inc = single[0];
        self.decode_ands[14].conduct_into(&[upper_incdec, op_lsb], &mut single)?;
        let enable_dec = single[0];

        let enables: [Signal; 8] = [
            enable_add, enable_sub, enable_and, enable_or, enable_xor, enable_not, enable_inc,
            enable_dec,
        ];

        // ---- Arithmetic path ---------------------------------------
        //
        // B' = B XOR sub, then A + B' + sub:
        //
        //   ADD -> A + B  + 0
        //   SUB -> A + NOT(B) + 1
        let mut modified_b = [Signal::HighImpedance; 8];

        for index in 0..8 {
            self.b_xors[index]
                .conduct_into(&[b[index], enable_sub], &mut modified_b[index..index + 1])?;
        }

        // INC forces B' = 00000001, DEC forces B' = 11111111; both
        // cascade after the SUB inversion level.
        let mut inc_or_arith = [Signal::HighImpedance; 8];
        let mut dec_or_all = [Signal::HighImpedance; 8];
        let mut mux_in = [Signal::HighImpedance; 3];
        let inc_consts: [Signal; 8] = [
            Signal::from(Bit::High),
            Signal::from(Bit::Low),
            Signal::from(Bit::Low),
            Signal::from(Bit::Low),
            Signal::from(Bit::Low),
            Signal::from(Bit::Low),
            Signal::from(Bit::Low),
            Signal::from(Bit::Low),
        ];
        let high = Signal::from(Bit::High);

        for index in 0..8 {
            mux_in[0] = modified_b[index];
            mux_in[1] = inc_consts[index];
            mux_in[2] = enable_inc;
            self.inc_muxes[index].conduct_into(&mux_in, &mut inc_or_arith[index..index + 1])?;

            mux_in[0] = inc_or_arith[index];
            mux_in[1] = high;
            mux_in[2] = enable_dec;
            self.dec_muxes[index].conduct_into(&mux_in, &mut dec_or_all[index..index + 1])?;
        }

        let mut adder_inputs = [Signal::HighImpedance; 17];
        adder_inputs[..8].copy_from_slice(a);
        adder_inputs[8..16].copy_from_slice(&dec_or_all);
        adder_inputs[16] = enable_sub;

        let mut adder_output = [Signal::HighImpedance; 9];
        self.adder.conduct_into(&adder_inputs, &mut adder_output)?;
        let sum = &adder_output[0..8];
        let raw_carry = adder_output[8];

        // ---- Logic path: one candidate bit per function slot ---------
        //
        // Slots 0 and 1 both expose the adder sum (ADD and SUB select
        // the same value; only their tri-state enables differ), then
        // AND, OR, XOR and NOT follow. INC and DEC reuse the adder
        // sum as well (slots 6 and 7).
        let mut candidates = [[Signal::HighImpedance; 8]; 8];

        for index in 0..8 {
            candidates[index][0] = sum[index];
            candidates[index][1] = sum[index];
            self.and_gates[index].conduct_into(&[a[index], b[index]], &mut single)?;
            candidates[index][2] = single[0];
            self.or_gates[index].conduct_into(&[a[index], b[index]], &mut single)?;
            candidates[index][3] = single[0];
            self.xor_gates[index].conduct_into(&[a[index], b[index]], &mut single)?;
            candidates[index][4] = single[0];
            self.not_gates[index].conduct_into(&[a[index]], &mut single)?;
            candidates[index][5] = single[0];
            candidates[index][6] = sum[index];
            candidates[index][7] = sum[index];
        }

        // ---- Tri-state bus: exactly one driver per bit --------------
        let mut drivers = [[Signal::HighImpedance; 8]; 8];

        for (function, enable) in enables.iter().enumerate() {
            for (index, slot) in drivers[function].iter_mut().enumerate() {
                self.output_nmos[index][function]
                    .conduct_into(&[*enable, candidates[index][function]], &mut single)?;
                *slot = single[0];
            }
        }

        let resolved = bus8(drivers)?;

        // ---- Flags: always valid, independently of `load` -----------
        //
        // The carry drives through the arithmetic select (raw adder
        // carry) or through the grounded pull-down wired below it
        // (logic operations clear it, like the 8080): exactly one of
        // the two NMOS rows conducts, so the flag never floats. The
        // zero flag is computed on the resolved internal result.
        self.zero_detector.conduct_into(&resolved, &mut single)?;
        let zero_flag = single[0];
        self.arith_or
            .conduct_into(&[enable_add, enable_sub], &mut single)?;
        let carry_low = single[0];
        self.arith_or_inc_dec
            .conduct_into(&[enable_inc, enable_dec], &mut single)?;
        let carry_high = single[0];
        self.arith_or_final
            .conduct_into(&[carry_low, carry_high], &mut single)?;
        let carry_enable = single[0];
        self.carry_nmos
            .conduct_into(&[carry_enable, raw_carry], &mut single)?;
        let carry_arith = single[0];
        self.logic_or_low
            .conduct_into(&[enable_and, enable_or], &mut single)?;
        let logic_low = single[0];
        self.logic_or_high
            .conduct_into(&[enable_xor, enable_not], &mut single)?;
        let logic_high = single[0];
        self.logic_or
            .conduct_into(&[logic_low, logic_high], &mut single)?;
        let logic_enable = single[0];
        self.carry_pulldown
            .conduct_into(&[logic_enable, Signal::Driven(Bit::Low)], &mut single)?;
        let carry_flag = wire([carry_arith, single[0]])?;

        // ---- Output buffer gated by `load` --------------------------
        for (index, nmos) in self.load_buffer_nmos.iter().enumerate() {
            nmos.conduct_into(&[load, resolved[index]], &mut outputs[index..index + 1])?;
        }

        outputs[8] = carry_flag;
        outputs[9] = zero_flag;

        Ok(())
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
            + sum_all!(self.inc_muxes)
            + sum_all!(self.dec_muxes)
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
            + self.arith_or_inc_dec.transistor_count()
            + self.arith_or_final.transistor_count()
            + self.logic_or_low.transistor_count()
            + self.logic_or_high.transistor_count()
            + self.logic_or.transistor_count()
            + self.carry_pulldown.transistor_count()
            + self.zero_detector.transistor_count()
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
    fn test_alu_sap1_subtraction() {
        use crate::utils::{bits_to_int, int_to_bits};

        let alu = AluSap1::default();

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

    #[test]
    fn test_alu_sap1_addition_exhaustive() {
        let alu = AluSap1::default();

        // All ADD pairs against the reference semantics.
        for a_value in 0..=255_u8 {
            let mut inputs: Vec<Bit> = Vec::with_capacity(18);
            inputs.extend(int_to_bits(a_value));
            inputs.extend(int_to_bits(137));
            inputs.extend([Bit::Low, Bit::High]); // subtract=Low, load=High
            let result = alu.compute(&inputs).expect("ALU computation failed");

            let total = u16::from(a_value) + 137_u16;
            assert_eq!(
                bits_to_int(&result[0..8], false),
                (total & 0xFF) as u8,
                "ADD failed for {a_value} + 137"
            );
            assert_eq!(result[8] == Bit::High, total > 0xFF, "carry for {a_value}");
        }
    }

    #[test]
    fn test_alu_sap2_operation_encodings() {
        use AluOperation::*;

        assert_eq!(Add.to_bits(), [Bit::Low, Bit::Low, Bit::Low]);
        assert_eq!(Sub.to_bits(), [Bit::Low, Bit::Low, Bit::High]);
        assert_eq!(And.to_bits(), [Bit::Low, Bit::High, Bit::Low]);
        assert_eq!(Or.to_bits(), [Bit::Low, Bit::High, Bit::High]);
        assert_eq!(Xor.to_bits(), [Bit::High, Bit::Low, Bit::Low]);
        assert_eq!(Not.to_bits(), [Bit::High, Bit::Low, Bit::High]);
        assert_eq!(Inc.to_bits(), [Bit::High, Bit::High, Bit::Low]);
        assert_eq!(Dec.to_bits(), [Bit::High, Bit::High, Bit::High]);
    }

    #[test]
    fn test_alu_sap2_increment_and_decrement() {
        use AluOperation::{Dec, Inc};

        let alu = AluSap2::default();

        // 0xFF + 1 = 0x00, carry out, zero set.
        assert_eq!(alu.evaluate(0xFF, 42, Inc, true).unwrap(), (0, true, true));

        // 7 + 1 = 8, no carry, B ignored.
        assert_eq!(alu.evaluate(7, 200, Inc, true).unwrap(), (8, false, false));

        // 0 - 1 = 0xFF, carry Low (borrow), sign bit set.
        let (result, carry, zero) = alu.evaluate(0, 0, Dec, true).unwrap();
        assert_eq!((result, carry, zero), (255, false, false));

        // 5 - 1 = 4, carry High (no borrow).
        assert_eq!(alu.evaluate(5, 200, Dec, true).unwrap(), (4, true, false));
    }

    #[test]
    fn test_alu_sap2_addition() {
        use AluOperation::Add;

        let alu = AluSap2::default();

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
    fn test_alu_sap2_subtraction() {
        use AluOperation::Sub;

        let alu = AluSap2::default();

        // 9 - 4 = 5, carry High (no borrow).
        assert_eq!(alu.evaluate(9, 4, Sub, true).unwrap(), (5, true, false));

        // 4 - 9 = 251 (wrapping), carry Low (borrow).
        assert_eq!(alu.evaluate(4, 9, Sub, true).unwrap(), (251, false, false));

        // 7 - 7 = 0, carry High, zero High.
        assert_eq!(alu.evaluate(7, 7, Sub, true).unwrap(), (0, true, true));
    }

    #[test]
    fn test_alu_sap2_logic_operations() {
        use AluOperation::{And, Not, Or, Xor};

        let alu = AluSap2::default();

        let (result, carry, _) = alu.evaluate(0b1100_1010, 0b1010_1100, And, true).unwrap();
        assert_eq!(result, 0b1000_1000);
        assert!(!carry); // logic ops do not drive the carry

        let (result, _, _) = alu.evaluate(0b1100_1010, 0b1010_1100, Or, true).unwrap();
        assert_eq!(result, 0b1110_1110);

        let (result, _, _) = alu.evaluate(0b1100_1010, 0b1010_1100, Xor, true).unwrap();
        assert_eq!(result, 0b0110_0110);

        let (result, _, _) = alu.evaluate(0b1100_1010, 0xFF, Not, true).unwrap();
        assert_eq!(result, 0b0011_0101);
    }

    #[test]
    fn test_alu_sap2_load_gates_the_result_but_not_flags() {
        use AluOperation::Add;

        let alu = AluSap2::default();

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
    fn test_alu_sap2_rejects_wrong_input_count() {
        let alu = AluSap2::default();
        assert_eq!(
            alu.conduct(&[Signal::HighImpedance; 19]),
            Err(HardwareError::InvalidInputCount {
                expected: 20,
                actual: 19,
            })
        );
    }

    #[test]
    fn test_alu_sap2_exhaustive_against_reference() {
        use AluOperation::*;

        let alu = AluSap2::default();

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

    #[test]
    fn test_sign_detector() {
        let detector = SignDetector8Bits::default();

        // Positive values: bit 7 Low.
        assert_eq!(detector.compute(&int_to_bits(0)).unwrap(), vec![Bit::Low]);
        assert_eq!(detector.compute(&int_to_bits(127)).unwrap(), vec![Bit::Low]);

        // Negative values (two's complement): bit 7 High.
        assert_eq!(
            detector.compute(&int_to_bits(128)).unwrap(),
            vec![Bit::High]
        );
        assert_eq!(
            detector.compute(&int_to_bits(255)).unwrap(),
            vec![Bit::High]
        );
    }
}
