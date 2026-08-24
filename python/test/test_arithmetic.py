"""
test_arithmetic.py
==================

Tests for the adders and the ALU in ``pytranscpu/arithmetic.py``.
"""

from __future__ import annotations

import pytest

from pytranscpu.arithmetic import Adder8Bits, ALU8Bits, FullAdder, HalfAdder
from pytranscpu.hardware import Bit, bits_to_int, int_to_bits


class TestHalfAdder:
    @pytest.mark.parametrize(
        ("a", "b", "expected"),
        [
            (0, 0, (0, 0)),
            (0, 1, (1, 0)),
            (1, 0, (1, 0)),
            (1, 1, (0, 1)),
        ],
    )
    def test_truth_table(self, a: Bit, b: Bit, expected: tuple[Bit, Bit]) -> None:
        assert HalfAdder()(a, b) == expected

    def test_transistor_count(self) -> None:
        assert HalfAdder().transistor_count == 22


class TestFullAdder:
    @pytest.mark.parametrize(
        ("a", "b", "carry_in", "expected"),
        [
            (0, 0, 0, (0, 0)),
            (0, 1, 0, (1, 0)),
            (1, 0, 0, (1, 0)),
            (1, 1, 0, (0, 1)),
            (0, 0, 1, (1, 0)),
            (0, 1, 1, (0, 1)),
            (1, 0, 1, (0, 1)),
            (1, 1, 1, (1, 1)),
        ],
    )
    def test_truth_table(
        self, a: Bit, b: Bit, carry_in: Bit, expected: tuple[Bit, Bit]
    ) -> None:
        assert FullAdder()(a, b, carry_in) == expected

    def test_transistor_count(self) -> None:
        assert FullAdder().transistor_count == 50


class TestAdder8Bits:
    @pytest.mark.parametrize(
        ("a", "b", "expected_sum", "expected_carry"),
        [
            (0, 0, 0, 0),
            (68, 40, 108, 0),
            (1, 255, 0, 1),
            (255, 1, 0, 1),
            (128, 128, 0, 1),
            (100, 155, 255, 0),
            (170, 85, 255, 0),
        ],
    )
    def test_addition(
        self, a: int, b: int, expected_sum: int, expected_carry: Bit
    ) -> None:
        sum_bits, carry = Adder8Bits()(int_to_bits(a, 8), int_to_bits(b, 8))

        assert bits_to_int(sum_bits) == expected_sum
        assert carry == expected_carry

    def test_matches_python_arithmetic_on_a_sample(self) -> None:
        adder = Adder8Bits()

        for a in range(0, 256, 16):
            for b in range(0, 256, 16):
                sum_bits, carry = adder(int_to_bits(a, 8), int_to_bits(b, 8))
                assert bits_to_int(sum_bits) + carry * 256 == a + b

    def test_carry_in(self) -> None:
        sum_bits, carry = Adder8Bits()(int_to_bits(255, 8), int_to_bits(0, 8), 1)
        assert bits_to_int(sum_bits) == 0
        assert carry == 1

        sum_bits, carry = Adder8Bits()(int_to_bits(3, 8), int_to_bits(4, 8), 1)
        assert bits_to_int(sum_bits) == 8
        assert carry == 0

    def test_transistor_count(self) -> None:
        assert Adder8Bits().transistor_count == 400


class TestALU8Bits:
    def test_addition(self) -> None:
        result, carry_flag, zero_flag = ALU8Bits()(
            int_to_bits(68, 8), int_to_bits(40, 8), 0, 1
        )
        assert result == int_to_bits(108, 8)
        assert carry_flag == 0
        assert zero_flag == 0

    def test_addition_with_carry(self) -> None:
        result, carry_flag, zero_flag = ALU8Bits()(
            int_to_bits(255, 8), int_to_bits(1, 8), 0, 1
        )
        assert result == int_to_bits(0, 8)
        assert carry_flag == 1
        assert zero_flag == 1

    def test_subtraction_without_borrow(self) -> None:
        result, carry_flag, zero_flag = ALU8Bits()(
            int_to_bits(108, 8), int_to_bits(40, 8), 1, 1
        )
        assert result == int_to_bits(68, 8)
        assert carry_flag == 1  # no borrow: 108 >= 40
        assert zero_flag == 0

    def test_subtraction_with_borrow(self) -> None:
        result, carry_flag, zero_flag = ALU8Bits()(
            int_to_bits(3, 8), int_to_bits(5, 8), 1, 1
        )
        assert result == int_to_bits(254, 8)  # -2 in two's complement
        assert carry_flag == 0  # borrow: 3 < 5
        assert zero_flag == 0

    def test_subtraction_to_zero(self) -> None:
        result, carry_flag, zero_flag = ALU8Bits()(
            int_to_bits(5, 8), int_to_bits(5, 8), 1, 1
        )
        assert result == int_to_bits(0, 8)
        assert carry_flag == 1
        assert zero_flag == 1

    def test_addition_matches_python_on_a_sample(self) -> None:
        alu = ALU8Bits()

        for a in range(0, 256, 32):
            for b in range(0, 256, 32):
                result, carry_flag, zero_flag = alu(
                    int_to_bits(a, 8), int_to_bits(b, 8), 0, 1
                )
                assert result == int_to_bits((a + b) % 256, 8)
                assert carry_flag == (1 if a + b > 255 else 0)
                assert zero_flag == (1 if (a + b) % 256 == 0 else 0)

    def test_subtraction_matches_python_on_a_sample(self) -> None:
        alu = ALU8Bits()

        for a in range(0, 256, 32):
            for b in range(0, 256, 32):
                result, carry_flag, zero_flag = alu(
                    int_to_bits(a, 8), int_to_bits(b, 8), 1, 1
                )
                assert result == int_to_bits((a - b) % 256, 8)
                assert carry_flag == (1 if a >= b else 0)
                assert zero_flag == (1 if a == b else 0)

    def test_load_low_floats_the_result_but_not_the_flags(self) -> None:
        result, carry_flag, zero_flag = ALU8Bits()(
            int_to_bits(255, 8), int_to_bits(1, 8), 0, 0
        )
        assert result == (None,) * 8
        assert carry_flag == 1
        assert zero_flag == 1

    def test_transistor_count(self) -> None:
        assert ALU8Bits().transistor_count == 580
