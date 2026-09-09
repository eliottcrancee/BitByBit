//! Architecture of the SAP-2 ("Simple As Possible 2") computer, from
//! Malvino's "Digital Computer Electronics", adapted to the hardware
//! primitives available in this project. [`Sap2`] is the CPU assembly
//! itself, wired from the decoder, sequencer and control unit below;
//! the micro-programs live in [`CONTROL_TABLE`].
//!
//! Compared to the SAP-1 (see [`crate::cpu_sap1`]), the SAP-2 adds:
//!
//! - a 16-bit address bus (64 KiB of RAM) while keeping an 8-bit data
//!   bus;
//! - multi-byte instructions (up to 3 bytes), so the fetch/execute
//!   cycle length depends on the instruction — the sequencer must
//!   support variable-length micro-programs;
//! - a second general-purpose register (B) and a full ALU (see
//!   [`crate::arithmetic::AluSap2`]) with AND, OR, XOR and NOT;
//! - a flags register extended with the sign flag (Z, C, S);
//! - immediate-mode instructions (MVI);
//! - I/O ports (IN / OUT);
//! - a hardware stack with CALL / RET / PUSH / POP and a 16-bit stack
//!   pointer.
//!
//! *Instructions* (the high bits form the opcode, followed by 0, 1 or
//! 2 operand bytes — the encoding below is the one to implement, the
//! hex values are placeholders until the decoder is designed):
//!
//! +-----------------+------+---------------------------------------------+
//! | Format          | Size | Instruction                                 |
//! +=================+======+=============================================+
//! | NOP             | 1    | NOP : no operation                          |
//! | LDA  a16        | 3    | LDA  : A <- MEM[a16]                        |
//! | STA  a16        | 3    | STA  : MEM[a16] <- A                        |
//! | MVI  A,d        | 2    | MVIA : A <- d (immediate)                   |
//! | MVI  B,d        | 2    | MVIB : B <- d (immediate)                   |
//! | ADD  B          | 1    | ADD  : A <- A + B                           |
//! | SUB  B          | 1    | SUB  : A <- A - B                           |
//! | ANA  B          | 1    | ANA  : A <- A AND B                         |
//! | ORA  B          | 1    | ORA  : A <- A OR B                          |
//! | XRA  B          | 1    | XRA  : A <- A XOR B                         |
//! | CMA             | 1    | CMA  : A <- NOT A (complement)              |
//! | INR  A          | 1    | INRA : A <- A + 1                           |
//! | DCR  A          | 1    | DCRA : A <- A - 1                           |
//! | IN              | 1    | IN   : A <- input port                      |
//! | OUT             | 1    | OUT  : output port <- A                     |
//! | JMP  a16        | 3    | JMP  : PC <- a16                            |
//! | JZ   a16        | 3    | JZ   : PC <- a16 if Z == 1                  |
//! | JNZ  a16        | 3    | JNZ  : PC <- a16 if Z == 0                  |
//! | JM   a16        | 3    | JM   : PC <- a16 if S == 1                  |
//! | CALL a16        | 3    | CALL: push PC, then PC <- a16               |
//! | RET             | 1    | RET  : PC <- pop()                          |
//! | PUSH A          | 1    | PSHA : push A onto the stack                |
//! | POP  A          | 1    | POPA : A <- pop()                           |
//! | HLT             | 1    | HLT  : halt the computer                    |
//! +-----------------+------+---------------------------------------------+
//!
//! *Components* — what must exist before the CPU can be wired:
//!
//! Already available in this project:
//!
//! - [`crate::gates`]          : AND, OR, XOR, NOT, NAND, NOR...;
//! - [`crate::mux`]            : 2:1 and 8-bit multiplexers;
//! - [`crate::decoder`]        : 2-to-4 and 4-to-16 decoders;
//! - [`crate::latches`]        : SR/D latches, D flip-flops, ring
//!   counter (sequencer core);
//! - [`crate::memory::Register8Bits`] : 8-bit registers (A, B, IR...);
//! - [`crate::arithmetic::Adder8Bits`] : ripple-carry adder;
//! - [`crate::arithmetic::AluSap2`] : ALU with ADD / SUB / AND / OR /
//!   XOR / NOT and carry + zero flags;
//! - [`crate::arithmetic::ZeroDetector8Bits`] : zero flag helper;
//! - [`crate::memory::Ram64KBits`] : 64 KiB RAM cascaded from 4096
//!   `Ram256Bits` chips;
//! - [`crate::memory::ProgramCounter16Bits`] : 16-bit counter with
//!   half-adder incrementer and per-half parallel load;
//! - [`crate::memory::MemoryAddressRegister16Bits`] : 16-bit MAR
//!   loaded in two halves from the 8-bit bus;
//! - [`crate::memory::FlagsRegister`] : zero, carry and sign latches;
//! - [`crate::arithmetic::SignDetector8Bits`] : sign flag detector;
//! - [`crate::memory::StackPointer16Bits`] : 16-bit up/down counter
//!   with per-half parallel load;
//! - [`crate::memory::InputPort`] : eight tri-state input switches;
//! - [`crate::memory::OutputRegister`] : the OUT display latch;
//! - [`InstructionDecoder`] : full opcode-byte decode matrix over 24
//!   one-hot instruction lines (encoding table below);
//! - [`Sequencer`] : free-running 4-bit T counter decoded into 16
//!   one-hot T lines, reloaded to T0 by the control unit's
//!   `seq_reset` (variable-length micro-programs, no branching);
//! - [`ControlUnit`] : PLA-style AND/OR matrix over T-states x
//!   decoded instruction x flags, driving all 29 control lines
//!   (micro-programs listed in [`CONTROL_TABLE`]).
//!
//! To be programmed (each as its own gate-level component, mirroring
//! the existing style — no software branching, tri-state outputs on
//! the shared bus, `transistor_count` everywhere):
//!
//! - `Sap2` — the CPU assembly itself (see below): every datapath
//!   component (PC16, SP16, MAR16, IR/A/B registers, ALU, flags,
//!   ports, RAM), the tri-state data/address buses resolved after
//!   every control word, the clock pulsed, stop on `halt`.
//!
//! *Micro-architecture sketch* (same bus organization as the SAP-1,
//! extended with the new paths):
//!
//! ```text
//!            +--------------------------------------------------
//!            |            16-bit address bus (MAR -> RAM)
//!            |
//!   [PC 16]--+--(via MAR)         RAM 64K <- 8-bit bidirectional bus
//!   [SP 16]--+                          |
//!   [IR  8]  |    +---------------------+
//!   [A   8]--+----+  shared 8-bit data bus (tri-state drivers)
//!   [B   8]  |    |
//!   [ALU8]---+    +-- [IN port] [OUT reg]
//!   [FLAGS]  |
//!            +-- ControlUnit(T-state, opcode, Z/C/S) -> control lines
//! ```

use crate::arithmetic::{AluSap2, HalfAdder, SignDetector8Bits};
use crate::decoder::Decoder4to16;
use crate::gates::{AndGate, NotGate, OrGate};
use crate::hardware::{bus8, wire, Bit, Component, HardwareError, Nmos, Signal};
use crate::latches::{DFlipFlopSave, SRLatch};
use crate::memory::{
    FlagsRegister, InputPort, MemoryAddressRegister16Bits, OutputRegister, ProgramCounter16Bits,
    Ram64KBits, Register8Bits, StackPointer16Bits, RAM64K_ADDRESS_BITS,
};
use crate::mux::Mux2x1;
use crate::utils::{bits_to_int, int_to_bits};

/// Opcode encoding of the SAP-2 instruction set. The high nibble
/// selects the instruction class, the low nibble is either unused
/// (`----`, don't care) or an exact opcode extension (`0000`...).
///
/// +------+--------+-------------------------------------------------+
/// | Line | Opcode | Instruction                                     |
/// +======+========+=================================================+
/// | 0    | 00     | NOP                                             |
/// | 1    | 10     | LDA  a16                                        |
/// | 2    | 20     | STA  a16                                        |
/// | 3    | 30     | MVIA d                                          |
/// | 4    | 40     | MVIB d                                          |
/// | 5    | 50     | ADD  B                                          |
/// | 6    | 51     | SUB  B                                          |
/// | 7    | 52     | ANA  B                                          |
/// | 8    | 53     | ORA  B                                          |
/// | 9    | 54     | XRA  B                                          |
/// | 10   | 55     | CMA                                             |
/// | 11   | 56     | INRA                                            |
/// | 12   | 57     | DCRA                                            |
/// | 13   | 60     | IN                                              |
/// | 14   | 61     | OUT                                             |
/// | 15   | 70     | JMP  a16                                        |
/// | 16   | 80     | JZ   a16                                        |
/// | 17   | 90     | JNZ  a16                                        |
/// | 18   | A0     | JM   a16                                        |
/// | 19   | B0     | CALL a16                                        |
/// | 20   | C0     | RET                                             |
/// | 21   | C1     | PSHA                                            |
/// | 22   | C2     | POPA                                            |
/// | 23   | F0     | HLT                                             |
/// +------+--------+-------------------------------------------------+
pub const OPCODE_BITS: usize = 8;
pub const INSTRUCTION_COUNT: usize = 24;

// Output line indices, one per instruction.
pub const IN_NOP: usize = 0;
pub const IN_LDA: usize = 1;
pub const IN_STA: usize = 2;
pub const IN_MVIA: usize = 3;
pub const IN_MVIB: usize = 4;
pub const IN_ADD: usize = 5;
pub const IN_SUB: usize = 6;
pub const IN_ANA: usize = 7;
pub const IN_ORA: usize = 8;
pub const IN_XRA: usize = 9;
pub const IN_CMA: usize = 10;
pub const IN_INRA: usize = 11;
pub const IN_DCRA: usize = 12;
pub const IN_IN: usize = 13;
pub const IN_OUT: usize = 14;
pub const IN_JMP: usize = 15;
pub const IN_JZ: usize = 16;
pub const IN_JNZ: usize = 17;
pub const IN_JM: usize = 18;
pub const IN_CALL: usize = 19;
pub const IN_RET: usize = 20;
pub const IN_PSHA: usize = 21;
pub const IN_POPA: usize = 22;
pub const IN_HLT: usize = 23;

/// Per-instruction opcode pattern, most significant bit first.
/// `1` = the bit must be High, `0` = the bit must be Low, `-1` =
/// don't care. Unwired bits are physically unconnected to that
/// instruction's enable net, exactly like a real decoder matrix.
const OPCODE_PATTERNS: [[i8; OPCODE_BITS]; INSTRUCTION_COUNT] = [
    [0, 0, 0, 0, 0, 0, 0, 0],     // 00      NOP
    [0, 0, 0, 1, -1, -1, -1, -1], // 10   LDA
    [0, 0, 1, 0, -1, -1, -1, -1], // 20   STA
    [0, 0, 1, 1, -1, -1, -1, -1], // 30   MVIA
    [0, 1, 0, 0, -1, -1, -1, -1], // 40   MVIB
    [0, 1, 0, 1, 0, 0, 0, 0],     // 50       ADD
    [0, 1, 0, 1, 0, 0, 0, 1],     // 51       SUB
    [0, 1, 0, 1, 0, 0, 1, 0],     // 52       ANA
    [0, 1, 0, 1, 0, 0, 1, 1],     // 53       ORA
    [0, 1, 0, 1, 0, 1, 0, 0],     // 54       XRA
    [0, 1, 0, 1, 0, 1, 0, 1],     // 55       CMA
    [0, 1, 0, 1, 0, 1, 1, 0],     // 56       INRA
    [0, 1, 0, 1, 0, 1, 1, 1],     // 57       DCRA
    [0, 1, 1, 0, 0, 0, 0, 0],     // 60       IN
    [0, 1, 1, 0, 0, 0, 0, 1],     // 61       OUT
    [0, 1, 1, 1, -1, -1, -1, -1], // 70   JMP
    [1, 0, 0, 0, -1, -1, -1, -1], // 80   JZ
    [1, 0, 0, 1, -1, -1, -1, -1], // 90   JNZ
    [1, 0, 1, 0, -1, -1, -1, -1], // A0   JM
    [1, 0, 1, 1, -1, -1, -1, -1], // B0   CALL
    [1, 1, 0, 0, 0, 0, 0, 0],     // C0       RET
    [1, 1, 0, 0, 0, 0, 0, 1],     // C1       PSHA
    [1, 1, 0, 0, 0, 0, 1, 0],     // C2       POPA
    [1, 1, 1, 1, 0, 0, 0, 0],     // F0       HLT
];

/// Number of AND gates in the decode matrix: every instruction with
/// `w` wired opcode bits contributes `w - 1` gates of its AND chain.
const fn and_gate_count(patterns: &[[i8; OPCODE_BITS]; INSTRUCTION_COUNT]) -> usize {
    let mut total = 0;
    let mut index = 0;
    while index < patterns.len() {
        let mut wired = 0;
        let mut bit = 0;
        while bit < OPCODE_BITS {
            if patterns[index][bit] >= 0 {
                wired += 1;
            }
            bit += 1;
        }
        total += wired - 1;
        index += 1;
    }
    total
}

/// The SAP-2 instruction decoder.
///
/// Eight inverters produce the complementary opcode nets; each of the
/// 24 instruction lines is the output of an AND chain wired to either
/// the direct net or its complement for every wired opcode bit, and
/// left unconnected for the don't-care bits. Exactly like the SAP-1's
/// control matrix, but over a full opcode byte: no software branching
/// anywhere.
impl Default for InstructionDecoder {
    fn default() -> Self {
        Self {
            nots: std::array::from_fn(|_| NotGate::default()),
            ands: std::array::from_fn(|_| AndGate::default()),
        }
    }
}

#[derive(Debug)]
pub struct InstructionDecoder {
    nots: [NotGate; OPCODE_BITS],
    ands: [AndGate; and_gate_count(&OPCODE_PATTERNS)],
}

impl Component for InstructionDecoder {
    /// Inputs:
    ///
    /// - 0..8: the opcode byte, least significant bit first
    ///
    /// Outputs:
    ///
    /// - 0..24: one-hot instruction lines (see the module table). An
    ///   opcode that matches no pattern leaves every line Low.
    const INPUTS: usize = OPCODE_BITS;
    const OUTPUTS: usize = INSTRUCTION_COUNT;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        // nets[0..8] = direct bits, nets[8..16] = complements.
        let mut nets = [Signal::HighImpedance; 2 * OPCODE_BITS];
        nets[..OPCODE_BITS].copy_from_slice(inputs);

        let mut single = [Signal::HighImpedance; 1];
        for (index, not_gate) in self.nots.iter().enumerate() {
            not_gate.conduct_into(&[inputs[index]], &mut single)?;
            nets[OPCODE_BITS + index] = single[0];
        }

        let mut and_offset = 0;
        for (instruction, pattern) in OPCODE_PATTERNS.iter().enumerate() {
            and_offset = Self::wire_chain(
                &nets,
                pattern,
                &self.ands,
                and_offset,
                &mut single,
                &mut outputs[instruction],
            )?;
        }

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.nots
            .iter()
            .map(|gate| gate.transistor_count())
            .sum::<usize>()
            + self
                .ands
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
    }
}

impl InstructionDecoder {
    /// Wire one instruction's AND chain over its pattern and drive
    /// `output`. Returns the offset of the next unused AND gate.
    fn wire_chain(
        nets: &[Signal; 2 * OPCODE_BITS],
        pattern: &[i8; OPCODE_BITS],
        gates: &[AndGate],
        mut offset: usize,
        single: &mut [Signal; 1],
        output: &mut Signal,
    ) -> Result<usize, HardwareError> {
        let mut acc: Option<Signal> = None;

        for (position, &required) in pattern.iter().enumerate() {
            // `position` walks the pattern MSB-first; the nets are
            // LSB-first, so the opcode bit is `OPCODE_BITS - 1 - position`.
            let bit = OPCODE_BITS - 1 - position;
            let net = match required {
                1 => nets[bit],
                0 => nets[OPCODE_BITS + bit],
                _ => continue,
            };

            acc = Some(match acc {
                None => net,
                Some(previous) => {
                    gates[offset].conduct_into(&[previous, net], single)?;
                    offset += 1;
                    single[0]
                }
            });
        }

        *output = acc.expect("every instruction wires at least one opcode bit");
        Ok(offset)
    }
}

/// Number of T-states generated by the [`Sequencer`].
pub const T_STATES: usize = 16;

/// Number of output lines of the [`ControlUnit`]: 26 named control
/// signals plus the 3 ALU operation bits.
pub const CONTROL_LINES: usize = 29;

// Control line indices (see `ControlUnit::conduct_into`).
pub const LN_PC_ENABLE: usize = 0;
pub const LN_SP_ENABLE: usize = 1;
pub const LN_MAR_ENABLE: usize = 2;
pub const LN_MAR_IN: usize = 3;
pub const LN_MAR_LO_IN: usize = 4;
pub const LN_MAR_HI_IN: usize = 5;
pub const LN_PC_COUNT: usize = 6;
pub const LN_PC_LOAD: usize = 7;
pub const LN_PC_SAVE_LO: usize = 8;
pub const LN_PC_SAVE_HI: usize = 9;
pub const LN_PC_OUT_LO: usize = 10;
pub const LN_PC_OUT_HI: usize = 11;
pub const LN_SP_COUNT_UP: usize = 12;
pub const LN_SP_COUNT_DOWN: usize = 13;
pub const LN_MEM_READ: usize = 14;
pub const LN_MEM_WRITE: usize = 15;
pub const LN_IR_IN: usize = 16;
pub const LN_ACC_IN: usize = 17;
pub const LN_ACC_OUT: usize = 18;
pub const LN_B_IN: usize = 19;
pub const LN_ALU_OUT: usize = 20;
pub const LN_FLAGS_IN: usize = 21;
pub const LN_IN_ENABLE: usize = 22;
pub const LN_OUT_SAVE: usize = 23;
pub const LN_HALT: usize = 24;
pub const LN_SEQ_RESET: usize = 25;
pub const LN_OP0: usize = 26;
pub const LN_OP1: usize = 27;
pub const LN_OP2: usize = 28;

/// Sentinel instruction index meaning "every instruction" (the fetch
/// micro-operations common to the whole instruction set).
const ANY_INSTR: u8 = 255;

/// Flag qualifier of a control term: unconditional, or gated by the
/// zero/sign flag (direct or inverted).
const FLAG_NONE: u8 = 0;
const FLAG_Z: u8 = 1;
const FLAG_NOT_Z: u8 = 2;
const FLAG_S: u8 = 3;

/// One AND term of the control matrix: `(T-state, instruction,
/// flag)`. The line fires when this T-state is active AND this
/// instruction is decoded AND the flag condition holds. With
/// `ANY_INSTR` the instruction AND gate is simply not wired.
#[derive(Debug)]
pub struct ControlTerm {
    pub t: u8,
    pub instr: u8,
    pub flag: u8,
}

const fn term(t: u8, instr: u8) -> ControlTerm {
    ControlTerm {
        t,
        instr,
        flag: FLAG_NONE,
    }
}

const fn term_flag(t: u8, instr: u8, flag: u8) -> ControlTerm {
    ControlTerm { t, instr, flag }
}

/// The micro-program: for every control line, the list of AND terms
/// wired into its OR tree.
///
/// Cycle numbering (one cycle = one rising clock edge):
///
/// - T0: fetch — the PC drives the address bus, the MAR captures it,
///   the PC increments;
/// - T1: the RAM drives the data bus, the IR captures the opcode;
/// - T2..: execution, per instruction. Multi-byte instructions hold
///   their low operand byte in MAR.lo (T3) while the PC addresses
///   the high operand byte at T5 (T4 idle); MAR.hi captures it,
///   then T6.. executes.
///
/// An opcode that decodes to nothing only ever sees the T0/T1 fetch
/// terms, then the T counter wraps around: it behaves as a 16-cycle
/// NOP.
const CONTROL_TABLE: [&[ControlTerm]; CONTROL_LINES] = [
    // pc_enable: fetch, the first operand byte of the multi-byte
    // instructions (the PC drives the address bus while the MAR
    // captures it), and the second operand byte of the 3-byte
    // instructions (the PC addresses it while the MAR keeps the low
    // byte captured at T3).
    &[
        term(0, ANY_INSTR),
        term(2, IN_LDA as u8),
        term(2, IN_STA as u8),
        term(2, IN_MVIA as u8),
        term(2, IN_MVIB as u8),
        term(2, IN_JMP as u8),
        term(2, IN_JZ as u8),
        term(2, IN_JNZ as u8),
        term(2, IN_JM as u8),
        term(2, IN_CALL as u8),
        term(5, IN_LDA as u8),
        term(5, IN_STA as u8),
        term(5, IN_JMP as u8),
        term(5, IN_JZ as u8),
        term(5, IN_JNZ as u8),
        term(5, IN_JM as u8),
        term(5, IN_CALL as u8),
    ],
    // sp_enable: stack access (the SP drives the address bus).
    &[
        term(2, IN_PSHA as u8),
        term(2, IN_POPA as u8),
        term(2, IN_RET as u8),
        term(4, IN_RET as u8),
        term(7, IN_CALL as u8),
    ],
    // mar_enable: computed separately (complement of the two lines
    // above), so the table entry is empty.
    &[],
    // mar_in (16-bit capture of the address bus): fetch and stack
    // access. The operand bytes of the multi-byte instructions use
    // the half captures below instead, so T4 stays idle with the MAR
    // holding the low byte.
    &[
        term(0, ANY_INSTR),
        term(2, IN_LDA as u8),
        term(2, IN_STA as u8),
        term(2, IN_MVIA as u8),
        term(2, IN_MVIB as u8),
        term(2, IN_JMP as u8),
        term(2, IN_JZ as u8),
        term(2, IN_JNZ as u8),
        term(2, IN_JM as u8),
        term(2, IN_CALL as u8),
        term(2, IN_PSHA as u8),
        term(2, IN_POPA as u8),
        term(2, IN_RET as u8),
        term(4, IN_RET as u8),
        term(7, IN_CALL as u8),
    ],
    // mar_lo_in: first operand byte (low half of the address) from
    // the data bus.
    &[
        term(3, IN_LDA as u8),
        term(3, IN_STA as u8),
        term(3, IN_JMP as u8),
        term(3, IN_JZ as u8),
        term(3, IN_JNZ as u8),
        term(3, IN_JM as u8),
        term(3, IN_CALL as u8),
    ],
    // mar_hi_in: second operand byte (high half of the address),
    // while the PC counts up to the next instruction.
    &[
        term(5, IN_LDA as u8),
        term(5, IN_STA as u8),
        term(5, IN_JMP as u8),
        term(5, IN_JZ as u8),
        term(5, IN_JNZ as u8),
        term(5, IN_JM as u8),
        term(5, IN_CALL as u8),
    ],
    // pc_count: fetch, both operand bytes of the 3-byte instructions,
    // and the single operand byte of the immediates.
    &[
        term(0, ANY_INSTR),
        term(2, IN_LDA as u8),
        term(2, IN_STA as u8),
        term(2, IN_MVIA as u8),
        term(2, IN_MVIB as u8),
        term(2, IN_JMP as u8),
        term(2, IN_JZ as u8),
        term(2, IN_JNZ as u8),
        term(2, IN_JM as u8),
        term(2, IN_CALL as u8),
        term(5, IN_LDA as u8),
        term(5, IN_STA as u8),
        term(5, IN_JMP as u8),
        term(5, IN_JZ as u8),
        term(5, IN_JNZ as u8),
        term(5, IN_JM as u8),
        term(5, IN_CALL as u8),
    ],
    // pc_load (16-bit capture of the address bus = jump): CALL copies
    // the target first so it can push the return address afterwards;
    // the conditional jumps gate the term with their flag.
    &[
        term(6, IN_JMP as u8),
        term_flag(6, IN_JZ as u8, FLAG_Z),
        term_flag(6, IN_JNZ as u8, FLAG_NOT_Z),
        term_flag(6, IN_JM as u8, FLAG_S),
        term(6, IN_CALL as u8),
    ],
    // pc_save_lo / pc_save_hi: RET pops the return address from the
    // data bus, high byte first (it sits at the lower address).
    &[term(5, IN_RET as u8)],
    &[term(3, IN_RET as u8)],
    // pc_out_lo / pc_out_hi: CALL pushes the return address onto the
    // data bus, low byte first.
    &[term(8, IN_CALL as u8)],
    &[term(9, IN_CALL as u8)],
    // sp_count_up: every popped byte releases one stack slot.
    &[
        term(3, IN_RET as u8),
        term(5, IN_RET as u8),
        term(3, IN_POPA as u8),
    ],
    // sp_count_down: every pushed byte reserves one stack slot.
    &[
        term(3, IN_PSHA as u8),
        term(8, IN_CALL as u8),
        term(9, IN_CALL as u8),
    ],
    // mem_read: opcode fetch, operand fetches, immediates, LDA, POPA, RET.
    &[
        term(1, ANY_INSTR),
        term(3, IN_LDA as u8),
        term(3, IN_STA as u8),
        term(3, IN_JMP as u8),
        term(3, IN_JZ as u8),
        term(3, IN_JNZ as u8),
        term(3, IN_JM as u8),
        term(3, IN_CALL as u8),
        term(3, IN_MVIA as u8),
        term(3, IN_MVIB as u8),
        term(5, IN_LDA as u8),
        term(5, IN_STA as u8),
        term(5, IN_JMP as u8),
        term(5, IN_JZ as u8),
        term(5, IN_JNZ as u8),
        term(5, IN_JM as u8),
        term(5, IN_CALL as u8),
        term(6, IN_LDA as u8),
        term(3, IN_POPA as u8),
        term(3, IN_RET as u8),
        term(5, IN_RET as u8),
    ],
    // mem_write: STA, PSHA and the two CALL push cycles.
    &[
        term(6, IN_STA as u8),
        term(3, IN_PSHA as u8),
        term(8, IN_CALL as u8),
        term(9, IN_CALL as u8),
    ],
    // ir_in: opcode capture.
    &[term(1, ANY_INSTR)],
    // acc_in: immediate loads, memory loads, IN, ALU results, pops.
    &[
        term(3, IN_MVIA as u8),
        term(3, IN_POPA as u8),
        term(6, IN_LDA as u8),
        term(2, IN_IN as u8),
        term(2, IN_ADD as u8),
        term(2, IN_SUB as u8),
        term(2, IN_ANA as u8),
        term(2, IN_ORA as u8),
        term(2, IN_XRA as u8),
        term(2, IN_CMA as u8),
        term(2, IN_INRA as u8),
        term(2, IN_DCRA as u8),
    ],
    // acc_out: STA, OUT and PSHA.
    &[
        term(6, IN_STA as u8),
        term(2, IN_OUT as u8),
        term(3, IN_PSHA as u8),
    ],
    // b_in: MVIB.
    &[term(3, IN_MVIB as u8)],
    // alu_out: every accumulator-modifying ALU operation.
    &[
        term(2, IN_ADD as u8),
        term(2, IN_SUB as u8),
        term(2, IN_ANA as u8),
        term(2, IN_ORA as u8),
        term(2, IN_XRA as u8),
        term(2, IN_CMA as u8),
        term(2, IN_INRA as u8),
        term(2, IN_DCRA as u8),
    ],
    // flags_in: ALU operations that update the flags (CMA does not).
    &[
        term(2, IN_ADD as u8),
        term(2, IN_SUB as u8),
        term(2, IN_ANA as u8),
        term(2, IN_ORA as u8),
        term(2, IN_XRA as u8),
        term(2, IN_INRA as u8),
        term(2, IN_DCRA as u8),
    ],
    // in_enable: IN.
    &[term(2, IN_IN as u8)],
    // out_save: OUT.
    &[term(2, IN_OUT as u8)],
    // halt: HLT.
    &[term(2, IN_HLT as u8)],
    // seq_reset: last cycle of every micro-program. Undefined opcodes
    // never assert it: the T counter wraps around instead.
    &[
        term(2, IN_NOP as u8),
        term(2, IN_HLT as u8),
        term(2, IN_ADD as u8),
        term(2, IN_SUB as u8),
        term(2, IN_ANA as u8),
        term(2, IN_ORA as u8),
        term(2, IN_XRA as u8),
        term(2, IN_CMA as u8),
        term(2, IN_INRA as u8),
        term(2, IN_DCRA as u8),
        term(2, IN_IN as u8),
        term(2, IN_OUT as u8),
        term(3, IN_MVIA as u8),
        term(3, IN_MVIB as u8),
        term(3, IN_PSHA as u8),
        term(3, IN_POPA as u8),
        term(5, IN_RET as u8),
        term(6, IN_LDA as u8),
        term(6, IN_STA as u8),
        term(6, IN_JMP as u8),
        term_flag(6, IN_JZ as u8, FLAG_Z),
        term_flag(6, IN_JNZ as u8, FLAG_NOT_Z),
        term_flag(6, IN_JM as u8, FLAG_S),
        term(9, IN_CALL as u8),
    ],
    // ALU operation bits (see `AluOperation::to_bits`): op0.
    &[
        term(2, IN_SUB as u8),
        term(2, IN_ORA as u8),
        term(2, IN_CMA as u8),
        term(2, IN_DCRA as u8),
    ],
    // op1.
    &[
        term(2, IN_ANA as u8),
        term(2, IN_ORA as u8),
        term(2, IN_INRA as u8),
        term(2, IN_DCRA as u8),
    ],
    // op2.
    &[
        term(2, IN_XRA as u8),
        term(2, IN_CMA as u8),
        term(2, IN_INRA as u8),
        term(2, IN_DCRA as u8),
    ],
];

/// AND gates of the decode matrix: every term costs one gate per
/// wired qualifier beyond the T-state net itself (instruction line,
/// then flag line). The two `ANY_INSTR` unconditional fetch terms
/// wire the T net directly.
const fn control_and_count() -> usize {
    let mut total = 0;
    let mut line = 0;
    while line < CONTROL_LINES {
        let terms = CONTROL_TABLE[line];
        let mut index = 0;
        while index < terms.len() {
            if terms[index].instr != ANY_INSTR {
                total += 1;
            }
            if terms[index].flag != FLAG_NONE {
                total += 1;
            }
            index += 1;
        }
        line += 1;
    }
    total
}

/// OR gates: every line with `n` terms contributes `n - 1` gates of
/// its combining tree (plus the `mar_enable` complement tree).
const fn control_or_count() -> usize {
    let mut total = 1; // the mar_enable OR of pc_enable/sp_enable
    let mut line = 0;
    while line < CONTROL_LINES {
        let terms = CONTROL_TABLE[line];
        if terms.len() > 1 {
            total += terms.len() - 1;
        }
        line += 1;
    }
    total
}

/// The SAP-2 control unit: the decoded micro-state (one-hot T lines),
/// the decoded opcode (one-hot instruction lines) and the flags feed
/// a two-level AND/OR matrix that drives all 29 control lines.
///
/// Nothing here branches in software: the wiring is derived from
/// [`CONTROL_TABLE`] exactly the way a programmable logic array is
/// programmed — every term is a physical AND gate, every combination
/// a physical OR tree.
#[derive(Debug)]
pub struct ControlUnit {
    /// Inverted zero flag, for the JNZ term.
    not_zero: NotGate,
    /// Complement tree: the MAR drives the address bus only when
    /// neither the PC nor the SP does.
    mar_or: OrGate,
    mar_not: NotGate,
    ands: [AndGate; control_and_count()],
    ors: [OrGate; control_or_count()],
}

impl Default for ControlUnit {
    fn default() -> Self {
        Self {
            not_zero: NotGate::default(),
            mar_or: OrGate::default(),
            mar_not: NotGate::default(),
            ands: std::array::from_fn(|_| AndGate::default()),
            ors: std::array::from_fn(|_| OrGate::default()),
        }
    }
}

impl Component for ControlUnit {
    /// Inputs:
    ///
    /// - 0..16: T-states, one-hot (from the [`Sequencer`])
    /// - 16..40: instruction lines, one-hot (from the
    ///   [`InstructionDecoder`])
    /// - 40: zero flag
    /// - 41: carry flag (unused by this matrix, wired for symmetry)
    /// - 42: sign flag
    ///
    /// Outputs:
    ///
    /// - 0..29: control lines, see the `LN_*` constants. Every line
    ///   is driven (High or Low) — these are enable inputs of the
    ///   datapath components, not bus wires.
    const INPUTS: usize = 16 + INSTRUCTION_COUNT + 3;
    const OUTPUTS: usize = CONTROL_LINES;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let t_nets = &inputs[0..16];
        let instr_nets = &inputs[16..16 + INSTRUCTION_COUNT];
        let zero = inputs[40];
        let sign = inputs[42];

        let mut single = [Signal::HighImpedance; 1];
        self.not_zero.conduct_into(&[zero], &mut single)?;
        let not_zero = single[0];

        let mut and_offset = 0;
        let mut or_offset = 0;

        for (line, terms) in CONTROL_TABLE.iter().enumerate() {
            // Evaluate every term of this line.
            let mut term_nets = [Signal::Driven(Bit::Low); 24];
            let mut term_count = 0;

            for spec in terms.iter() {
                let mut net = t_nets[spec.t as usize];

                if spec.instr != ANY_INSTR {
                    self.ands[and_offset]
                        .conduct_into(&[net, instr_nets[spec.instr as usize]], &mut single)?;
                    net = single[0];
                    and_offset += 1;
                }

                match spec.flag {
                    FLAG_NONE => {}
                    FLAG_Z => {
                        self.ands[and_offset].conduct_into(&[net, zero], &mut single)?;
                        net = single[0];
                        and_offset += 1;
                    }
                    FLAG_NOT_Z => {
                        self.ands[and_offset].conduct_into(&[net, not_zero], &mut single)?;
                        net = single[0];
                        and_offset += 1;
                    }
                    _ => {
                        self.ands[and_offset].conduct_into(&[net, sign], &mut single)?;
                        net = single[0];
                        and_offset += 1;
                    }
                }

                term_nets[term_count] = net;
                term_count += 1;
            }

            // mar_enable (empty entry): computed after the loop as the
            // complement of pc_enable OR sp_enable.
            if term_count == 0 {
                continue;
            }

            // Combine the terms with an OR tree (single terms pass
            // through directly).
            let mut acc = term_nets[0];
            for term_net in term_nets[1..term_count].iter() {
                self.ors[or_offset].conduct_into(&[acc, *term_net], &mut single)?;
                acc = single[0];
                or_offset += 1;
            }

            outputs[line] = acc;
        }

        // mar_enable = NOT(pc_enable OR sp_enable): the address bus
        // never has two drivers.
        self.mar_or
            .conduct_into(&[outputs[LN_PC_ENABLE], outputs[LN_SP_ENABLE]], &mut single)?;
        self.mar_not
            .conduct_into(&[single[0]], &mut outputs[LN_MAR_ENABLE..LN_MAR_ENABLE + 1])?;

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.not_zero.transistor_count()
            + self.mar_or.transistor_count()
            + self.mar_not.transistor_count()
            + self
                .ands
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
            + self
                .ors
                .iter()
                .map(|gate| gate.transistor_count())
                .sum::<usize>()
    }
}

/// The SAP-2 sequencer: a free-running 4-bit T-state counter whose
/// outputs are decoded into [`T_STATES`] one-hot lines.
///
/// On every rising clock edge the counter advances by one, unless
/// `reset` is High — then it reloads zero, which restarts the fetch.
/// The reset decision comes combinationally from the control unit
/// (`seq_reset`), so micro-programs of different lengths all share
/// this single counter: a variable-length sequencer with no software
/// branching.
///
/// Power-on state is all-zeros, i.e. T0: no initialization hardware
/// is needed.
#[derive(Debug, Default)]
pub struct Sequencer {
    flip_flops: [DFlipFlopSave; 4],
    /// Half-adder chain computing `state + 1` (carry-in High).
    half_adders: [HalfAdder; 4],
    /// Per bit: next count, or zero when `reset` is High.
    reset_muxes: [Mux2x1; 4],
    decoder: Decoder4to16,
}

impl Sequencer {
    /// The stored 4-bit count, least significant bit first.
    pub fn state(&self) -> [Bit; 4] {
        let mut value = [Bit::Low; 4];
        for (index, flip_flop) in self.flip_flops.iter().enumerate() {
            value[index] = flip_flop.q();
        }
        value
    }
}

impl Component for Sequencer {
    /// Inputs:
    ///
    /// - 0: clock
    /// - 1: reset — reload T0 on the rising edge when High
    ///
    /// Outputs:
    ///
    /// - 0..16: one-hot T lines (exactly one High).
    const INPUTS: usize = 2;
    const OUTPUTS: usize = T_STATES;

    fn conduct_into(&self, inputs: &[Signal], outputs: &mut [Signal]) -> Result<(), HardwareError> {
        if inputs.len() != Self::INPUTS {
            return Err(HardwareError::InvalidInputCount {
                expected: Self::INPUTS,
                actual: inputs.len(),
            });
        }

        let clock = inputs[0];
        let reset = inputs[1];

        let stored = self.state();
        let mut state = [Signal::HighImpedance; 4];
        for (index, bit) in stored.iter().enumerate() {
            state[index] = Signal::from(*bit);
        }

        // Incremented = state + 1 over the 4 bits: a ripple of half
        // adders starting with a carry-in of High.
        let mut carry = Signal::Driven(Bit::High);
        let mut ha_out = [Signal::HighImpedance; 2];
        let mut next = [Signal::HighImpedance; 4];
        let mut mux_in = [Signal::HighImpedance; 3];

        for (index, flip_flop) in self.flip_flops.iter().enumerate() {
            self.half_adders[index].conduct_into(&[state[index], carry], &mut ha_out)?;
            carry = ha_out[1];

            // reset High forces the bit to zero, whatever the count.
            mux_in[0] = ha_out[0];
            mux_in[1] = Signal::Driven(Bit::Low);
            mux_in[2] = reset;
            self.reset_muxes[index].conduct_into(&mux_in, &mut next[index..index + 1])?;

            // Capture on every rising edge: the counter always runs.
            // The flip-flop drives both q and q_bar; only q is exposed.
            let mut both = [Signal::HighImpedance; 2];
            flip_flop.conduct_into(&[next[index], clock, Signal::Driven(Bit::High)], &mut both)?;
        }

        self.decoder.conduct_into(&state, outputs)?;

        Ok(())
    }

    fn transistor_count(&self) -> usize {
        self.flip_flops
            .iter()
            .map(|flip_flop| flip_flop.transistor_count())
            .sum::<usize>()
            + self
                .half_adders
                .iter()
                .map(|adder| adder.transistor_count())
                .sum::<usize>()
            + self
                .reset_muxes
                .iter()
                .map(|mux| mux.transistor_count())
                .sum::<usize>()
            + self.decoder.transistor_count()
    }
}

#[cfg(test)]
mod sequencer_tests {
    use super::*;
    use crate::hardware::Bit;
    use Bit::{High, Low};

    fn pulse(seq: &Sequencer, reset: Bit) -> Vec<Bit> {
        let run = |clock: Bit| seq.compute(&[clock, reset]).expect("tick failed");

        run(Low);
        run(High);
        // The settle tick decodes the freshly captured state.
        run(Low)
    }

    #[test]
    fn test_power_on_state_is_t0() {
        let seq = Sequencer::default();
        let outputs = seq.compute(&[Low, Low]).expect("compute failed");
        assert_eq!(outputs[0], High);
        assert_eq!(outputs.iter().filter(|b| **b == High).count(), 1);
    }

    #[test]
    fn test_advances_and_wraps_around() {
        let seq = Sequencer::default();

        // From T0, 16 pulses visit T1..T15 then wrap back to T0.
        let mut visited = vec![0];
        for _ in 0..16 {
            let outputs = pulse(&seq, Low);
            visited.push(
                outputs
                    .iter()
                    .position(|b| *b == High)
                    .expect("no active T line"),
            );
        }
        let expected: Vec<usize> = (0..17).map(|t| t % 16).collect();
        assert_eq!(visited, expected);
    }

    #[test]
    fn test_reset_reloads_t0() {
        let seq = Sequencer::default();

        // Advance to count 3 = binary 0011.
        for _ in 0..3 {
            pulse(&seq, Low);
        }
        assert_eq!(seq.state(), [High, High, Low, Low]);

        // A pulse with reset High lands back on T0.
        pulse(&seq, High);
        assert_eq!(seq.state(), [Low, Low, Low, Low]);
        let outputs = seq.compute(&[Low, Low]).expect("compute failed");
        assert_eq!(outputs[0], High);
    }

    #[test]
    fn test_transistor_count() {
        // 4 x DFlipFlopSave (66) + 4 x HalfAdder (22) + 4 x Mux2x1 (20)
        // + Decoder4to16 (152).
        assert_eq!(Sequencer::default().transistor_count(), 584);
    }
}

#[cfg(test)]
mod control_unit_tests {
    use super::*;
    use crate::hardware::Bit;
    use crate::utils::int_to_bits;
    use Bit::{High, Low};

    /// Run the control unit for one micro-state and return the list
    /// of High control lines.
    fn lines(unit: &ControlUnit, t: usize, opcode: u8, zero: Bit, sign: Bit) -> Vec<usize> {
        let decoder = InstructionDecoder::default();
        let mut inputs: Vec<Bit> = vec![Low; 16];
        inputs[t] = High;
        inputs.extend(
            decoder
                .compute(&int_to_bits(opcode))
                .expect("decode failed"),
        );
        inputs.extend([zero, Low, sign]);

        let outputs = unit.compute(&inputs).expect("control unit failed");
        let mut high = outputs
            .iter()
            .enumerate()
            .filter(|(_, line)| **line == High)
            .map(|(index, _)| index)
            .collect::<Vec<usize>>();
        high.sort_unstable();
        high
    }

    /// The two unconditional fetch cycles, common to every opcode.
    /// While the PC drives the address bus (T0), the MAR is disabled.
    #[test]
    fn test_fetch_cycles() {
        let unit = ControlUnit::default();

        assert_eq!(
            lines(&unit, 0, 0x00, Low, Low),
            vec![LN_PC_ENABLE, LN_MAR_IN, LN_PC_COUNT]
        );
        assert_eq!(
            lines(&unit, 1, 0x00, Low, Low),
            vec![LN_MAR_ENABLE, LN_MEM_READ, LN_IR_IN]
        );
    }

    #[test]
    fn test_lda_micro_program() {
        let unit = ControlUnit::default();

        assert_eq!(
            lines(&unit, 2, 0x10, Low, Low),
            vec![LN_PC_ENABLE, LN_MAR_IN, LN_PC_COUNT]
        );
        assert_eq!(
            lines(&unit, 3, 0x10, Low, Low),
            vec![LN_MAR_ENABLE, LN_MAR_LO_IN, LN_MEM_READ]
        );
        assert_eq!(lines(&unit, 4, 0x10, Low, Low), vec![LN_MAR_ENABLE]);
        assert_eq!(
            lines(&unit, 5, 0x10, Low, Low),
            vec![LN_PC_ENABLE, LN_MAR_HI_IN, LN_PC_COUNT, LN_MEM_READ],
        );
        assert_eq!(
            lines(&unit, 6, 0x10, Low, Low),
            vec![LN_MAR_ENABLE, LN_MEM_READ, LN_ACC_IN, LN_SEQ_RESET]
        );
    }

    #[test]
    fn test_alu_operations_drive_the_op_lines() {
        let unit = ControlUnit::default();

        // ADD = 000: no op line.
        assert_eq!(
            lines(&unit, 2, 0x50, Low, Low),
            vec![
                LN_MAR_ENABLE,
                LN_ACC_IN,
                LN_ALU_OUT,
                LN_FLAGS_IN,
                LN_SEQ_RESET
            ]
        );
        // SUB = 001: op0.
        assert!(lines(&unit, 2, 0x51, Low, Low).contains(&LN_OP0));
        // INRA = 110: op2 + op1.
        let inra = lines(&unit, 2, 0x56, Low, Low);
        assert!(inra.contains(&LN_OP1) && inra.contains(&LN_OP2) && !inra.contains(&LN_OP0));
        // DCRA = 111: all three.
        let dcra = lines(&unit, 2, 0x57, Low, Low);
        assert!(dcra.contains(&LN_OP0) && dcra.contains(&LN_OP1) && dcra.contains(&LN_OP2));
        // CMA = 101: no flags_in.
        assert!(!lines(&unit, 2, 0x55, Low, Low).contains(&LN_FLAGS_IN));
    }

    #[test]
    fn test_conditional_jumps_obey_the_flags() {
        let unit = ControlUnit::default();

        // JZ taken (zero High) and not taken (zero Low).
        assert_eq!(
            lines(&unit, 6, 0x80, High, Low),
            vec![LN_MAR_ENABLE, LN_PC_LOAD, LN_SEQ_RESET]
        );
        assert_eq!(lines(&unit, 6, 0x80, Low, Low), vec![LN_MAR_ENABLE]);

        // JNZ is the complement.
        assert_eq!(lines(&unit, 6, 0x90, High, Low), vec![LN_MAR_ENABLE]);
        assert_eq!(
            lines(&unit, 6, 0x90, Low, Low),
            vec![LN_MAR_ENABLE, LN_PC_LOAD, LN_SEQ_RESET]
        );

        // JM gates on the sign flag.
        assert_eq!(
            lines(&unit, 6, 0xA0, Low, High),
            vec![LN_MAR_ENABLE, LN_PC_LOAD, LN_SEQ_RESET]
        );
        assert_eq!(lines(&unit, 6, 0xA0, Low, Low), vec![LN_MAR_ENABLE]);
    }

    #[test]
    fn test_call_micro_program() {
        let unit = ControlUnit::default();

        let expected: [Vec<usize>; 10] = [
            vec![LN_PC_ENABLE, LN_MAR_IN, LN_PC_COUNT],
            vec![LN_MAR_ENABLE, LN_MEM_READ, LN_IR_IN],
            vec![LN_PC_ENABLE, LN_MAR_IN, LN_PC_COUNT],
            vec![LN_MAR_ENABLE, LN_MAR_LO_IN, LN_MEM_READ],
            vec![LN_MAR_ENABLE],
            vec![LN_PC_ENABLE, LN_MAR_HI_IN, LN_PC_COUNT, LN_MEM_READ],
            vec![LN_MAR_ENABLE, LN_PC_LOAD],
            vec![LN_SP_ENABLE, LN_MAR_IN],
            vec![LN_MAR_ENABLE, LN_PC_OUT_LO, LN_SP_COUNT_DOWN, LN_MEM_WRITE],
            vec![
                LN_MAR_ENABLE,
                LN_PC_OUT_HI,
                LN_SP_COUNT_DOWN,
                LN_MEM_WRITE,
                LN_SEQ_RESET,
            ],
        ];

        for (t, expected_lines) in expected.iter().enumerate() {
            assert_eq!(
                lines(&unit, t, 0xB0, Low, Low),
                *expected_lines,
                "CALL T{t}"
            );
        }
    }

    #[test]
    fn test_undefined_opcodes_only_fetch() {
        let unit = ControlUnit::default();

        assert_eq!(lines(&unit, 2, 0xD5, Low, Low), vec![LN_MAR_ENABLE]);
        assert_eq!(lines(&unit, 9, 0xE7, Low, Low), vec![LN_MAR_ENABLE]);
    }

    #[test]
    fn test_mar_enable_complement() {
        let unit = ControlUnit::default();

        // T0: the PC drives the address bus, the MAR must not.
        let mut inputs = vec![Low; 43];
        inputs[0] = High;
        inputs[16] = High;
        let outputs = unit.compute(&inputs).expect("control unit failed");
        assert_eq!(outputs[LN_MAR_ENABLE], Low);

        // T1: nobody else drives it, the MAR does.
        let mut inputs = vec![Low; 43];
        inputs[1] = High;
        inputs[16] = High;
        let outputs = unit.compute(&inputs).expect("control unit failed");
        assert_eq!(outputs[LN_MAR_ENABLE], High);
    }
}

/// Integration: sequencer + decoder + control unit running the CALL
/// micro-program end to end, with `seq_reset` wired back into the
/// sequencer — exactly the loop the full CPU will run.
#[cfg(test)]
mod sequencer_control_integration_tests {
    use super::*;
    use crate::hardware::Bit;
    use crate::utils::int_to_bits;
    use Bit::{High, Low};

    #[test]
    fn test_call_micro_program_cycles_through_the_sequencer() {
        let sequencer = Sequencer::default();
        let decoder = InstructionDecoder::default();
        let unit = ControlUnit::default();

        let mut visited = Vec::new();
        let mut resets = Vec::new();

        for _ in 0..14 {
            let t_lines = sequencer.compute(&[Low, Low]).expect("seq failed");
            let t = t_lines
                .iter()
                .position(|b| *b == High)
                .expect("no active T line");
            visited.push(t);

            let mut inputs: Vec<Bit> = t_lines;
            inputs.extend(decoder.compute(&int_to_bits(0xB0)).expect("decode failed"));
            inputs.extend([Low, Low, Low]);

            let control = unit.compute(&inputs).expect("control unit failed");
            if control[LN_SEQ_RESET] == High {
                resets.push(t);
            }

            let reset = control[LN_SEQ_RESET];
            sequencer.compute(&[Low, reset]).expect("low tick failed");
            sequencer.compute(&[High, reset]).expect("high tick failed");
            sequencer
                .compute(&[Low, reset])
                .expect("settle tick failed");
        }

        // T0..T9, then back to T0 for the next fetch.
        assert_eq!(visited[0..10], [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]);
        assert_eq!(visited[10..14], [0, 1, 2, 3]);
        assert_eq!(resets, vec![9]);
    }
}

#[cfg(test)]
mod instruction_decoder_tests {
    use super::*;
    use crate::hardware::Bit;
    use crate::utils::int_to_bits;

    /// (opcode byte, expected instruction line).
    const CANONICAL: [(u8, usize); INSTRUCTION_COUNT] = [
        (0x00, IN_NOP),
        (0x10, IN_LDA),
        (0x20, IN_STA),
        (0x30, IN_MVIA),
        (0x40, IN_MVIB),
        (0x50, IN_ADD),
        (0x51, IN_SUB),
        (0x52, IN_ANA),
        (0x53, IN_ORA),
        (0x54, IN_XRA),
        (0x55, IN_CMA),
        (0x56, IN_INRA),
        (0x57, IN_DCRA),
        (0x60, IN_IN),
        (0x61, IN_OUT),
        (0x70, IN_JMP),
        (0x80, IN_JZ),
        (0x90, IN_JNZ),
        (0xA0, IN_JM),
        (0xB0, IN_CALL),
        (0xC0, IN_RET),
        (0xC1, IN_PSHA),
        (0xC2, IN_POPA),
        (0xF0, IN_HLT),
    ];

    #[test]
    fn test_canonical_opcodes_decode_one_hot() {
        let decoder = InstructionDecoder::default();

        for (opcode, expected_line) in CANONICAL {
            let outputs = decoder
                .compute(&int_to_bits(opcode))
                .expect("decode failed");

            for (line, output) in outputs.iter().enumerate() {
                let expected = if line == expected_line {
                    Bit::High
                } else {
                    Bit::Low
                };
                assert_eq!(
                    *output, expected,
                    "opcode {opcode:#04x}: line {line} should be {expected:?}"
                );
            }
        }
    }

    #[test]
    fn test_dont_care_low_nibble_still_decodes() {
        let decoder = InstructionDecoder::default();

        // LDA is 0x1----: every low nibble decodes to LDA.
        for low in 0..16_u8 {
            let outputs = decoder
                .compute(&int_to_bits(0x10 | low))
                .expect("decode failed");
            assert_eq!(outputs[IN_LDA], Bit::High, "0x1{low:x} should be LDA");
        }
    }

    #[test]
    fn test_undefined_opcodes_activate_no_line() {
        let decoder = InstructionDecoder::default();

        // Gaps in the encoding space: extensions of ALU ops beyond
        // 0x57, the 0xD0 class, and C3..CF.
        for opcode in [0x58, 0x5F, 0x62, 0x6F, 0xD0, 0xD5, 0xC3, 0xCF, 0xFF] {
            let outputs = decoder
                .compute(&int_to_bits(opcode))
                .expect("decode failed");
            assert!(
                outputs.iter().all(|line| *line == Bit::Low),
                "opcode {opcode:#04x} must not activate any line"
            );
        }
    }
}

/// The SAP-2 computer: PC/SP/MAR (16-bit), IR/A/B (8-bit), ALU with
/// flags, 64 KiB RAM, sequencer + decoder + control unit, IN/OUT ports.
///
/// Every signal path below goes through transistors: bus drivers are
/// NMOS rows resolved with [`wire`] / [`bus8`], decoding and control
/// are gate nets, sequencing is flip-flops. The only Rust-level
/// decisions are the external engine (clock toggle, halt loop,
/// fetch-to-fetch stepping), mirroring [`crate::cpu_sap1::Sap1`].
/// No new `FAST` shortcut lives here: the components apply the
/// established ones themselves.
///
// ponytail: CALL pushes both return bytes at the same MAR address
// and PUSH/POP are off by one slot (write-then-decrement vs
// read-after-increment); multi-byte stack programs need per-byte
// `mar_in` cycles plus an updated `test_call_micro_program`.
#[derive(Debug)]
pub struct Sap2 {
    pc: ProgramCounter16Bits,
    sp: StackPointer16Bits,
    mar: MemoryAddressRegister16Bits,
    ir: Register8Bits,
    acc: Register8Bits,
    b: Register8Bits,
    alu: AluSap2,
    sign_detector: SignDetector8Bits,
    flags: FlagsRegister,
    ram: Ram64KBits,
    sequencer: Sequencer,
    decoder: InstructionDecoder,
    control_unit: ControlUnit,
    in_port: InputPort,
    out_reg: OutputRegister,
    halt_latch: SRLatch,
    halt_and: AndGate,
    /// Tri-state address-bus rows: exactly one of PC / SP / MAR drives
    /// the 16 address wires (`mar_enable` is the wired NOR of the rest).
    pc_addr_nmos: [Nmos; 16],
    sp_addr_nmos: [Nmos; 16],
    mar_addr_nmos: [Nmos; 16],
    /// Tri-state data-bus rows for the PC halves and the accumulator.
    /// The RAM, ALU and input port drive through their own rows.
    pc_lo_nmos: [Nmos; 8],
    pc_hi_nmos: [Nmos; 8],
    acc_bus_nmos: [Nmos; 8],
    /// Physical input switches sampled by the IN instruction.
    input_value: [Bit; 8],
    halted: bool,
    clock_signal: Bit,
}

impl Default for Sap2 {
    fn default() -> Self {
        let cpu = Self {
            pc: ProgramCounter16Bits::default(),
            sp: StackPointer16Bits::default(),
            mar: MemoryAddressRegister16Bits::default(),
            ir: Register8Bits::default(),
            acc: Register8Bits::default(),
            b: Register8Bits::default(),
            alu: AluSap2::default(),
            sign_detector: SignDetector8Bits::default(),
            flags: FlagsRegister::default(),
            ram: Ram64KBits::default(),
            sequencer: Sequencer::default(),
            decoder: InstructionDecoder::default(),
            control_unit: ControlUnit::default(),
            in_port: InputPort::default(),
            out_reg: OutputRegister::default(),
            halt_latch: SRLatch::default(),
            halt_and: AndGate::default(),
            pc_addr_nmos: std::array::from_fn(|_| Nmos::default()),
            sp_addr_nmos: std::array::from_fn(|_| Nmos::default()),
            mar_addr_nmos: std::array::from_fn(|_| Nmos::default()),
            pc_lo_nmos: std::array::from_fn(|_| Nmos::default()),
            pc_hi_nmos: std::array::from_fn(|_| Nmos::default()),
            acc_bus_nmos: std::array::from_fn(|_| Nmos::default()),
            input_value: [Bit::Low; 8],
            halted: false,
            clock_signal: Bit::Low,
        };
        // Park the stack at the top of memory: the ISA has no load-SP
        // instruction, so reset hardware presets 0xFFFF. Both halves
        // capture 0xFF on a single Low-High clock pulse.
        let low = Signal::Driven(Bit::Low);
        let high = Signal::Driven(Bit::High);
        let mut inputs = [low; StackPointer16Bits::INPUTS];
        inputs[..8].fill(high);
        inputs[11] = high;
        inputs[12] = high;
        cpu.sp
            .conduct_into(
                &inputs,
                &mut [Signal::HighImpedance; StackPointer16Bits::OUTPUTS],
            )
            .expect("SP reset preload failed");
        inputs[8] = high;
        cpu.sp
            .conduct_into(
                &inputs,
                &mut [Signal::HighImpedance; StackPointer16Bits::OUTPUTS],
            )
            .expect("SP reset capture failed");
        cpu
    }
}

impl Sap2 {
    /// Becomes true once a HLT instruction has been executed.
    pub fn halted(&self) -> bool {
        self.halted
    }

    /// The current value of the shared clock.
    pub fn clock(&self) -> Bit {
        self.clock_signal
    }

    /// The accumulator content, LSB first.
    pub fn out(&self) -> [Bit; 8] {
        self.acc.state()
    }

    /// The OUT display latch, LSB first.
    pub fn output(&self) -> [Bit; 8] {
        self.out_reg.state()
    }

    /// The stored flags as `(zero, carry, sign)`.
    pub fn flags(&self) -> (Bit, Bit, Bit) {
        self.flags.state()
    }

    /// The instruction currently stored in the instruction register.
    pub fn current_instruction(&self) -> u8 {
        bits_to_int(&self.ir.state(), false)
    }

    /// The 16-bit program counter value.
    pub fn pc_value(&self) -> u16 {
        let state = self.pc.state();
        bits_to_int(&state[..8], false) as u16 | ((bits_to_int(&state[8..], false) as u16) << 8)
    }

    /// The 16-bit memory address register value.
    pub fn mar_value(&self) -> u16 {
        let state = self.mar.state();
        bits_to_int(&state[..8], false) as u16 | ((bits_to_int(&state[8..], false) as u16) << 8)
    }

    /// The 4-bit T-state count, LSB first.
    pub fn t_state(&self) -> [Bit; 4] {
        self.sequencer.state()
    }

    /// Set the physical input switches sampled by IN.
    pub fn set_input(&mut self, value: u8) {
        self.input_value = int_to_bits(value);
    }

    /// Toggle the shared clock and return the new value.
    pub fn clock_tick(&mut self) -> Result<Bit, HardwareError> {
        if self.halted {
            return Ok(self.clock_signal);
        }
        self.clock_signal = match self.clock_signal {
            Bit::Low => Bit::High,
            Bit::High => Bit::Low,
        };
        self.update_at(self.clock_signal)?;
        Ok(self.clock_signal)
    }

    /// Update the internal circuit state with the current clock.
    pub fn update(&mut self) -> Result<(), HardwareError> {
        self.update_at(self.clock_signal)
    }

    /// Advance the sequencer by one micro-state and execute it.
    pub fn micro_step(&mut self) -> Result<(), HardwareError> {
        if self.halted {
            return Ok(());
        }
        if self.clock_signal == Bit::High {
            self.clock_tick()?;
        }
        self.clock_tick()?;
        self.clock_tick()?;
        Ok(())
    }

    /// Advance execution until the next instruction fetch starts or
    /// the machine halts.
    pub fn step_instruction(&mut self) -> Result<(), HardwareError> {
        if self.halted {
            return Ok(());
        }
        self.micro_step()?;
        while !self.halted && self.sequencer.state() != [Bit::Low; 4] {
            self.micro_step()?;
        }
        Ok(())
    }

    /// Run instructions until halted or `max_steps` instructions have
    /// been stepped.
    pub fn run(&mut self, max_steps: usize) -> Result<(), HardwareError> {
        let mut steps = 0;
        while !self.halted && steps < max_steps {
            self.step_instruction()?;
            steps += 1;
        }
        Ok(())
    }

    /// Load a sequence of bytes into RAM starting at address 0.
    pub fn load_program(&mut self, program: &[u8]) -> Result<(), HardwareError> {
        for (address, value) in program.iter().enumerate().take(1 << RAM64K_ADDRESS_BITS) {
            self.ram.write_byte(address, int_to_bits(*value))?;
        }
        Ok(())
    }

    /// Total number of transistors, every sub-component included.
    pub fn transistor_count(&self) -> usize {
        self.pc.transistor_count()
            + self.sp.transistor_count()
            + self.mar.transistor_count()
            + self.ir.transistor_count()
            + self.acc.transistor_count()
            + self.b.transistor_count()
            + self.alu.transistor_count()
            + self.sign_detector.transistor_count()
            + self.flags.transistor_count()
            + self.ram.transistor_count()
            + self.sequencer.transistor_count()
            + self.decoder.transistor_count()
            + self.control_unit.transistor_count()
            + self.in_port.transistor_count()
            + self.out_reg.transistor_count()
            + self.halt_latch.transistor_count()
            + self.halt_and.transistor_count()
            + self
                .pc_addr_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
            + self
                .sp_addr_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
            + self
                .mar_addr_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
            + self
                .pc_lo_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
            + self
                .pc_hi_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
            + self
                .acc_bus_nmos
                .iter()
                .map(|nmos| nmos.transistor_count())
                .sum::<usize>()
    }

    /// Clock a register on the rising edge; `load` selects whether its
    /// outputs drive the data bus (IR/B never do).
    fn clock_register(
        &self,
        register: &Register8Bits,
        data: &[Signal; 8],
        clock: Signal,
        save: Bit,
        load: Bit,
    ) -> Result<(), HardwareError> {
        let mut inputs = [Signal::HighImpedance; Register8Bits::INPUTS];
        inputs[..8].copy_from_slice(data);
        inputs[8] = clock;
        inputs[9] = Signal::from(save);
        inputs[10] = Signal::from(load);
        register.conduct_into(
            &inputs,
            &mut [Signal::HighImpedance; Register8Bits::OUTPUTS],
        )?;
        Ok(())
    }

    /// Propagate signals through the combinational and sequential parts
    /// of the machine for the given clock value.
    fn update_at(&mut self, clock: Bit) -> Result<(), HardwareError> {
        let clock_signal = Signal::from(clock);
        let low = Signal::from(Bit::Low);

        // Decode the stored micro-state without advancing it: reset Low
        // keeps the count, the live clock keeps the master preload
        // intact on High ticks and is overwritten by the real
        // propagation below on Low ticks.
        // ponytail: one extra sequencer propagation per tick; a latched
        // T-bus would remove it at the cost of new state.
        let t_lines = self.sequencer.compute(&[clock, Bit::Low])?;
        let instr_lines = self.decoder.compute(&self.ir.state())?;
        let (zero, carry, sign) = self.flags.state();

        let mut ctrl_in = [Bit::Low; ControlUnit::INPUTS];
        ctrl_in[..16].copy_from_slice(&t_lines);
        ctrl_in[16..16 + INSTRUCTION_COUNT].copy_from_slice(&instr_lines);
        ctrl_in[16 + INSTRUCTION_COUNT] = zero;
        ctrl_in[16 + INSTRUCTION_COUNT + 1] = carry;
        ctrl_in[16 + INSTRUCTION_COUNT + 2] = sign;
        let ctrl = self.control_unit.compute(&ctrl_in)?;

        // Address bus: one NMOS row per source, resolved as wires.
        // A row whose enable is Low floats by itself, so every row is
        // propagated unconditionally and only the active one drives.
        let pc_state = self.pc.state();
        let sp_state = self.sp.state();
        let mar_state = self.mar.state();
        let mut addr_bus = [Signal::HighImpedance; 16];
        for i in 0..16 {
            let pc_drv = self.pc_addr_nmos[i]
                .conduct(&[Signal::from(ctrl[LN_PC_ENABLE]), Signal::from(pc_state[i])])?[0];
            let sp_drv = self.sp_addr_nmos[i]
                .conduct(&[Signal::from(ctrl[LN_SP_ENABLE]), Signal::from(sp_state[i])])?[0];
            let mar_drv = self.mar_addr_nmos[i].conduct(&[
                Signal::from(ctrl[LN_MAR_ENABLE]),
                Signal::from(mar_state[i]),
            ])?[0];
            addr_bus[i] = wire([pc_drv, sp_drv, mar_drv])?;
        }

        // Data bus drivers (before the RAM): PC halves, ACC, ALU, IN.
        let acc_state = self.acc.state();
        let b_state = self.b.state();
        let mut pc_lo = [Signal::HighImpedance; 8];
        let mut pc_hi = [Signal::HighImpedance; 8];
        let mut acc_row = [Signal::HighImpedance; 8];
        for i in 0..8 {
            pc_lo[i] = self.pc_lo_nmos[i]
                .conduct(&[Signal::from(ctrl[LN_PC_OUT_LO]), Signal::from(pc_state[i])])?[0];
            pc_hi[i] = self.pc_hi_nmos[i].conduct(&[
                Signal::from(ctrl[LN_PC_OUT_HI]),
                Signal::from(pc_state[8 + i]),
            ])?[0];
            acc_row[i] = self.acc_bus_nmos[i]
                .conduct(&[Signal::from(ctrl[LN_ACC_OUT]), Signal::from(acc_state[i])])?[0];
        }

        let mut alu_inputs = [Signal::HighImpedance; AluSap2::INPUTS];
        for (index, bit) in acc_state.iter().enumerate() {
            alu_inputs[index] = Signal::from(*bit);
        }
        for (index, bit) in b_state.iter().enumerate() {
            alu_inputs[8 + index] = Signal::from(*bit);
        }
        alu_inputs[16] = Signal::from(ctrl[LN_OP2]);
        alu_inputs[17] = Signal::from(ctrl[LN_OP1]);
        alu_inputs[18] = Signal::from(ctrl[LN_OP0]);
        alu_inputs[19] = Signal::from(ctrl[LN_ALU_OUT]);
        let mut alu_outputs = [Signal::HighImpedance; AluSap2::OUTPUTS];
        self.alu.conduct_into(&alu_inputs, &mut alu_outputs)?;
        let mut alu_row = [Signal::HighImpedance; 8];
        alu_row.copy_from_slice(&alu_outputs[..8]);
        let carry_out = alu_outputs[8];
        let zero_out = alu_outputs[9];
        let mut sign_out = [Signal::HighImpedance; 1];
        self.sign_detector.conduct_into(&alu_row, &mut sign_out)?;

        let mut in_inputs = [Signal::HighImpedance; InputPort::INPUTS];
        for (index, bit) in self.input_value.iter().enumerate() {
            in_inputs[index] = Signal::from(*bit);
        }
        in_inputs[8] = Signal::from(ctrl[LN_IN_ENABLE]);
        let mut in_row = [Signal::HighImpedance; InputPort::OUTPUTS];
        self.in_port.conduct_into(&in_inputs, &mut in_row)?;

        let partial_bus = bus8([pc_lo, pc_hi, acc_row, alu_row, in_row])?;

        // RAM: one propagation through the memory itself performs the
        // write at the rising edge (`save` = MEM_WRITE) and drives the
        // bus (`load` = MEM_READ). MEM_WRITE and MEM_READ are never
        // both High in the micro-code, so the RAM never fights the
        // other drivers.
        let mut ram_inputs = [Signal::HighImpedance; Ram64KBits::INPUTS];
        ram_inputs[..8].copy_from_slice(&partial_bus);
        ram_inputs[8..24].copy_from_slice(&addr_bus);
        ram_inputs[24] = clock_signal;
        ram_inputs[25] = Signal::from(ctrl[LN_MEM_WRITE]);
        ram_inputs[26] = Signal::from(ctrl[LN_MEM_READ]);
        let mut ram_row = [Signal::HighImpedance; Ram64KBits::OUTPUTS];
        self.ram.conduct_into(&ram_inputs, &mut ram_row)?;

        let bus = bus8([partial_bus, ram_row])?;

        // Sequential clocking. Storage components whose `save` input
        // is Low hold their state (see `Register8Bits`); the counters
        // gate their own capture on count/load lines.
        let mut pc_inputs = [Signal::HighImpedance; ProgramCounter16Bits::INPUTS];
        pc_inputs[..8].copy_from_slice(&bus);
        pc_inputs[8..24].copy_from_slice(&addr_bus);
        pc_inputs[24] = clock_signal;
        pc_inputs[25] = Signal::from(ctrl[LN_PC_COUNT]);
        pc_inputs[26] = Signal::from(ctrl[LN_PC_LOAD]);
        pc_inputs[27] = Signal::from(ctrl[LN_PC_SAVE_LO]);
        pc_inputs[28] = Signal::from(ctrl[LN_PC_SAVE_HI]);
        pc_inputs[29] = Signal::from(ctrl[LN_PC_ENABLE]);
        pc_inputs[30] = Signal::from(ctrl[LN_PC_OUT_LO]);
        pc_inputs[31] = Signal::from(ctrl[LN_PC_OUT_HI]);
        self.pc.conduct_into(
            &pc_inputs,
            &mut [Signal::HighImpedance; ProgramCounter16Bits::OUTPUTS],
        )?;

        let mut sp_inputs = [Signal::HighImpedance; StackPointer16Bits::INPUTS];
        sp_inputs[..8].copy_from_slice(&bus);
        sp_inputs[8] = clock_signal;
        sp_inputs[9] = Signal::from(ctrl[LN_SP_COUNT_UP]);
        sp_inputs[10] = Signal::from(ctrl[LN_SP_COUNT_DOWN]);
        sp_inputs[11] = low;
        sp_inputs[12] = low;
        sp_inputs[13] = Signal::from(ctrl[LN_SP_ENABLE]);
        self.sp.conduct_into(
            &sp_inputs,
            &mut [Signal::HighImpedance; StackPointer16Bits::OUTPUTS],
        )?;

        let mut mar_inputs = [Signal::HighImpedance; MemoryAddressRegister16Bits::INPUTS];
        mar_inputs[..8].copy_from_slice(&bus);
        mar_inputs[8..24].copy_from_slice(&addr_bus);
        mar_inputs[24] = clock_signal;
        mar_inputs[25] = Signal::from(ctrl[LN_MAR_LO_IN]);
        mar_inputs[26] = Signal::from(ctrl[LN_MAR_HI_IN]);
        mar_inputs[27] = Signal::from(ctrl[LN_MAR_IN]);
        mar_inputs[28] = Signal::from(ctrl[LN_MAR_ENABLE]);
        self.mar.conduct_into(
            &mar_inputs,
            &mut [Signal::HighImpedance; MemoryAddressRegister16Bits::OUTPUTS],
        )?;

        self.clock_register(&self.ir, &bus, clock_signal, ctrl[LN_IR_IN], Bit::Low)?;
        self.clock_register(
            &self.acc,
            &bus,
            clock_signal,
            ctrl[LN_ACC_IN],
            ctrl[LN_ACC_OUT],
        )?;
        self.clock_register(&self.b, &bus, clock_signal, ctrl[LN_B_IN], Bit::Low)?;

        let mut flag_inputs = [Signal::HighImpedance; FlagsRegister::INPUTS];
        flag_inputs[0] = zero_out;
        flag_inputs[1] = carry_out;
        flag_inputs[2] = sign_out[0];
        flag_inputs[3] = clock_signal;
        flag_inputs[4] = Signal::from(ctrl[LN_FLAGS_IN]);
        self.flags.conduct_into(
            &flag_inputs,
            &mut [Signal::HighImpedance; FlagsRegister::OUTPUTS],
        )?;

        let mut out_inputs = [Signal::HighImpedance; OutputRegister::INPUTS];
        out_inputs[..8].copy_from_slice(&bus);
        out_inputs[8] = clock_signal;
        out_inputs[9] = Signal::from(ctrl[LN_OUT_SAVE]);
        self.out_reg.conduct_into(
            &out_inputs,
            &mut [Signal::HighImpedance; OutputRegister::OUTPUTS],
        )?;

        let halt_set = self.halt_and.compute(&[ctrl[LN_HALT], clock])?[0];
        self.halt_latch.conduct(&[Signal::from(halt_set), low])?;
        if self.halt_latch.q() == Bit::High {
            self.halted = true;
        }

        self.sequencer
            .conduct(&[clock_signal, Signal::from(ctrl[LN_SEQ_RESET])])?;

        Ok(())
    }
}

#[cfg(test)]
mod sap2_program_tests {
    use super::*;

    fn run_program(program: &[u8], max_steps: usize) -> Sap2 {
        let mut cpu = Sap2::default();
        cpu.load_program(program).expect("load failed");
        cpu.run(max_steps).expect("run failed");
        cpu
    }

    #[test]
    fn test_transistor_count() {
        assert!(Sap2::default().transistor_count() > 0);
    }

    #[test]
    fn test_stack_parks_at_top_of_memory() {
        let cpu = Sap2::default();
        assert_eq!(cpu.sp.state(), [Bit::High; 16]);
    }

    #[test]
    fn test_mvia_and_hlt() {
        let cpu = run_program(&[0x30, 10, 0xF0], 100); // MVIA 10, HLT
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 10);
    }

    #[test]
    fn test_add() {
        // MVIB 3, MVIA 5, ADD B, HLT
        let cpu = run_program(&[0x40, 3, 0x30, 5, 0x50, 0xF0], 100);
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 8);
    }

    #[test]
    fn test_sub() {
        // MVIB 4, MVIA 9, SUB B, HLT
        let cpu = run_program(&[0x40, 4, 0x30, 9, 0x51, 0xF0], 100);
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 5);
    }

    #[test]
    fn test_logic_and_complement() {
        // ANA: A=10 (1010), B=12 (1100) -> 8
        let cpu = run_program(&[0x40, 12, 0x30, 10, 0x52, 0xF0], 100);
        assert_eq!(bits_to_int(&cpu.out(), false), 8);
        // ORA -> 14
        let cpu = run_program(&[0x40, 12, 0x30, 10, 0x53, 0xF0], 100);
        assert_eq!(bits_to_int(&cpu.out(), false), 14);
        // XRA -> 6
        let cpu = run_program(&[0x40, 12, 0x30, 10, 0x54, 0xF0], 100);
        assert_eq!(bits_to_int(&cpu.out(), false), 6);
        // CMA: ~0x55 -> 0xAA
        let cpu = run_program(&[0x30, 0x55, 0x55, 0xF0], 100);
        assert_eq!(bits_to_int(&cpu.out(), false), 0xAA);
        // INRA 7 -> 8, DCRA 7 -> 6
        let cpu = run_program(&[0x30, 7, 0x56, 0xF0], 100);
        assert_eq!(bits_to_int(&cpu.out(), false), 8);
        let cpu = run_program(&[0x30, 7, 0x57, 0xF0], 100);
        assert_eq!(bits_to_int(&cpu.out(), false), 6);
    }

    #[test]
    fn test_lda_and_sta() {
        // MVIA 7, STA 0x0010, MVIA 0, LDA 0x0010, HLT
        let cpu = run_program(
            &[0x30, 7, 0x20, 0x10, 0x00, 0x30, 0, 0x10, 0x10, 0x00, 0xF0],
            100,
        );
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 7);
    }

    #[test]
    fn test_jmp() {
        // 0: JMP 6, 3: MVIA 9 (skipped), 5: HLT (skipped),
        // 6: MVIA 4, 8: HLT
        let cpu = run_program(&[0x70, 6, 0, 0x30, 9, 0xF0, 0x30, 4, 0xF0], 100);
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 4);
    }

    #[test]
    fn test_jz_taken_and_not_taken() {
        // MVIB 5, MVIA 5, SUB (Z=1), JZ 11, MVIA 9, HLT, MVIA 6, HLT
        let taken = vec![
            0x40, 5, 0x30, 5, 0x51, 0x80, 11, 0, 0x30, 9, 0xF0, 0x30, 6, 0xF0,
        ];
        let cpu = run_program(&taken, 100);
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 6);
        // MVIB 3, MVIA 5, SUB (Z=0), JZ 11 not taken -> MVIA 9, HLT
        let not_taken = vec![
            0x40, 3, 0x30, 5, 0x51, 0x80, 11, 0, 0x30, 9, 0xF0, 0x30, 6, 0xF0,
        ];
        let cpu = run_program(&not_taken, 100);
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 9);
    }

    #[test]
    fn test_in_and_out() {
        let mut cpu = Sap2::default();
        cpu.set_input(23);
        cpu.load_program(&[0x60, 0xF0]).expect("load failed"); // IN, HLT
        cpu.run(100).expect("run failed");
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 23);

        let cpu = run_program(&[0x30, 42, 0x61, 0xF0], 100); // MVIA 42, OUT, HLT
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.output(), false), 42);
    }

    #[test]
    fn test_halted_clock_stops() {
        let mut cpu = Sap2::default();
        cpu.load_program(&[0xF0]).expect("load failed"); // HLT only
        cpu.run(10).expect("run failed");
        assert!(cpu.halted());
        let clock_before = cpu.clock();
        cpu.clock_tick().expect("clock tick failed");
        assert_eq!(cpu.clock(), clock_before);
    }
}
