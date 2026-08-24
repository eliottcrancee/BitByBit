"""
test_mux.py
===========

Tests for the multiplexers in ``pytranscpu/mux.py``.
"""

from __future__ import annotations

import pytest

from pytranscpu.hardware import Bit
from pytranscpu.mux import Mux2x1, Mux8bits2x1


class TestMux2x1:
    @pytest.mark.parametrize(
        ("a", "b", "select", "expected"),
        [
            (0, 0, 0, 0),
            (0, 1, 0, 0),
            (1, 0, 0, 1),
            (1, 1, 0, 1),
            (0, 0, 1, 0),
            (0, 1, 1, 1),
            (1, 0, 1, 0),
            (1, 1, 1, 1),
        ],
    )
    def test_truth_table(self, a: Bit, b: Bit, select: Bit, expected: Bit) -> None:
        assert Mux2x1()(a, b, select) == expected

    def test_transistor_count(self) -> None:
        assert Mux2x1().transistor_count == 20

    def test_repr(self) -> None:
        assert str(Mux2x1()) == "Mux2x1(transistors=20, memory_bits=0)"


class TestMux8bits2x1:
    def test_selects_a_when_low(self) -> None:
        assert (
            Mux8bits2x1()((0, 1, 0, 1, 0, 1, 0, 1), (1, 0, 1, 0, 1, 0, 1, 0), 0)
            == (0, 1, 0, 1, 0, 1, 0, 1)
        )

    def test_selects_b_when_high(self) -> None:
        assert (
            Mux8bits2x1()((0, 1, 0, 1, 0, 1, 0, 1), (1, 0, 1, 0, 1, 0, 1, 0), 1)
            == (1, 0, 1, 0, 1, 0, 1, 0)
        )

    def test_transistor_count(self) -> None:
        assert Mux8bits2x1().transistor_count == 160

    def test_repr(self) -> None:
        assert str(Mux8bits2x1()) == "Mux8bits2x1(transistors=160, memory_bits=0)"
