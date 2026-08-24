"""
test_cpu_sap1.py
================

Tests for the SAP-1 computer implementation in ``pytranscpu/cpu_sap1.py``.
"""

from __future__ import annotations

from pytranscpu.cpu_sap1 import SAP1, ControlUnit
from pytranscpu.hardware import bits_to_int


class TestSAP1:
    def test_transistor_and_bit_counts(self) -> None:
        cpu = SAP1()
        assert cpu.transistor_count > 0
        assert cpu.bit_count > 0
        assert cpu.cost.transistors == cpu.transistor_count
        assert cpu.cost.memory_bits == cpu.bit_count

    def test_nop_instruction(self) -> None:
        cpu = SAP1()
        cpu.load_program([0x00, 0xF0])  # NOP, HLT
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 0

    def test_ldi_instruction(self) -> None:
        cpu = SAP1()
        cpu.load_program([0x5A, 0xF0])  # LDI 10, HLT
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 10

    def test_lda_and_sta_instructions(self) -> None:
        cpu = SAP1()
        # LDI 7, STA 0xA, LDI 0, LDA 0xA, HLT
        cpu.load_program([0x57, 0x4A, 0x50, 0x1A, 0xF0])
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 7

    def test_add_instruction(self) -> None:
        cpu = SAP1()
        # LDI 5, ADD 0xE, HLT
        program = [0x55, 0x2E, 0xF0] + [0] * 11 + [3, 0]  # mem[0xE] = 3
        cpu.load_program(program)
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 8

    def test_sub_instruction(self) -> None:
        cpu = SAP1()
        # LDI 9, SUB 0xE, HLT
        program = [0x59, 0x3E, 0xF0] + [0] * 11 + [4, 0]  # mem[0xE] = 4
        cpu.load_program(program)
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 5

    def test_jmp_instruction(self) -> None:
        cpu = SAP1()
        # 0: JMP 3
        # 1: LDI 9 (skipped)
        # 2: HLT
        # 3: LDI 4
        # 4: HLT
        cpu.load_program([0x63, 0x59, 0xF0, 0x54, 0xF0])
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 4

    def test_jz_taken(self) -> None:
        cpu = SAP1()
        # 0: LDI 3
        # 1: SUB 0xE (3 - 3 = 0, sets Zero flag)
        # 2: JZ 5 (branch taken)
        # 3: LDI 9 (skipped)
        # 4: HLT
        # 5: LDI 6
        # 6: HLT
        program = [0x53, 0x3E, 0x75, 0x59, 0xF0, 0x56, 0xF0] + [0] * 7 + [3, 0]
        cpu.load_program(program)
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 6

    def test_jz_not_taken(self) -> None:
        cpu = SAP1()
        # 0: LDI 4
        # 1: SUB 0xE (4 - 3 = 1, Zero flag = 0)
        # 2: JZ 5 (branch not taken)
        # 3: LDI 9
        # 4: HLT
        # 5: LDI 6
        # 6: HLT
        program = [0x54, 0x3E, 0x75, 0x59, 0xF0, 0x56, 0xF0] + [0] * 7 + [3, 0]
        cpu.load_program(program)
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 9

    def test_jc_taken(self) -> None:
        cpu = SAP1()
        # 0: LDI 10
        # 1: ADD 0xE (10 + 250 = 260 -> carry out = 1)
        # 2: JC 5 (branch taken)
        # 3: LDI 9 (skipped)
        # 4: HLT
        # 5: LDI 6
        # 6: HLT
        program = [0x5A, 0x2E, 0x85, 0x59, 0xF0, 0x56, 0xF0] + [0] * 7 + [250, 0]
        cpu.load_program(program)
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 6

    def test_jc_not_taken(self) -> None:
        cpu = SAP1()
        # 0: LDI 2
        # 1: ADD 0xE (2 + 3 = 5, carry out = 0)
        # 2: JC 5 (branch not taken)
        # 3: LDI 9
        # 4: HLT
        # 5: LDI 6
        # 6: HLT
        program = [0x52, 0x2E, 0x85, 0x59, 0xF0, 0x56, 0xF0] + [0] * 7 + [3, 0]
        cpu.load_program(program)
        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 9

    def test_loop_multiplication_program(self) -> None:
        """Multiply 3 by 5 using a loop."""
        cpu = SAP1()
        program = [
            0x1F,  # 0: LDA 0xF (load accumulator / running sum)
            0x2E,  # 1: ADD 0xE (add 3)
            0x4F,  # 2: STA 0xF (store back to running sum)
            0x1D,  # 3: LDA 0xD (load count)
            0x3C,  # 4: SUB 0xC (subtract 1)
            0x79,  # 5: JZ 9 (if count reached 0, jump to 9)
            0x4D,  # 6: STA 0xD (save decremented count)
            0x60,  # 7: JMP 0 (loop back to LDA 0xF)
            0x00,  # 8: NOP
            0x1F,  # 9: LDA 0xF (load final sum)
            0xF0,  # A: HLT
            0x00,  # B: unused
            0x01,  # C: constant 1
            0x05,  # D: initial count = 5
            0x03,  # E: constant 3
            0x00,  # F: sum initialized to 0
        ]
        cpu.load_program(program)
        cpu.run(max_steps=500)
        assert cpu.halted
        assert bits_to_int(cpu.out) == 15

    def test_clock_tick_and_micro_step(self) -> None:
        cpu = SAP1()
        cpu.load_program([0x57, 0xF0])  # LDI 7, HLT

        # Advance step by step using clock_tick
        assert cpu.clock_signal == 0
        assert cpu.clock_tick() == 1
        assert cpu.clock_tick() == 0

        # Run with micro_step
        cpu.micro_step()
        assert not cpu.halted

        cpu.run()
        assert cpu.halted
        assert bits_to_int(cpu.out) == 7


class TestControlUnit:
    def test_transistor_count(self) -> None:
        cu = ControlUnit()
        assert cu.transistor_count > 0
        assert cu.bit_count == 0
