"""
test_gates.py
=============

Tests for the logic gates in ``pytranscpu/gates.py``.
"""

from __future__ import annotations

from typing import Final, cast

import pytest

from pytranscpu.gates import (
    AndGate,
    NandGate,
    NorGate,
    NotGate,
    OrGate,
    XnorGate,
    XorGate,
)
from pytranscpu.hardware import Bit, InvalidSignalError

# Deliberately invalid signal: neither LOW (0) nor HIGH (1). Used only to
# check that gates reject out-of-range inputs with ``InvalidSignalError``.
# The ``cast`` is a white lie told to the type checker: ``Bit`` cannot
# represent this value, which is exactly the point of the test.
INVALID_BIT: Final[Bit] = cast(Bit, 2)


class TestNotGate:
    @pytest.mark.parametrize(("a", "expected"), [(0, 1), (1, 0)])
    def test_truth_table(self, a: Bit, expected: Bit) -> None:
        assert NotGate()(a) == expected

    def test_transistor_count(self) -> None:
        assert NotGate().transistor_count == 2

    def test_repr(self) -> None:
        assert str(NotGate()) == "NotGate(transistors=2, memory_bits=0)"


class TestNandGate:
    @pytest.mark.parametrize(
        ("a", "b", "expected"),
        [
            (0, 0, 1),
            (0, 1, 1),
            (1, 0, 1),
            (1, 1, 0),
        ],
    )
    def test_truth_table(self, a: Bit, b: Bit, expected: Bit) -> None:
        assert NandGate()(a, b) == expected

    def test_transistor_count(self) -> None:
        assert NandGate().transistor_count == 4

    def test_repr(self) -> None:
        assert str(NandGate()) == "NandGate(transistors=4, memory_bits=0)"


class TestNorGate:
    @pytest.mark.parametrize(
        ("a", "b", "expected"),
        [
            (0, 0, 1),
            (0, 1, 0),
            (1, 0, 0),
            (1, 1, 0),
        ],
    )
    def test_truth_table(self, a: Bit, b: Bit, expected: Bit) -> None:
        assert NorGate()(a, b) == expected

    def test_transistor_count(self) -> None:
        assert NorGate().transistor_count == 4

    def test_repr(self) -> None:
        assert str(NorGate()) == "NorGate(transistors=4, memory_bits=0)"


class TestAndGate:
    @pytest.mark.parametrize(
        ("a", "b", "expected"),
        [
            (0, 0, 0),
            (0, 1, 0),
            (1, 0, 0),
            (1, 1, 1),
        ],
    )
    def test_truth_table(self, a: Bit, b: Bit, expected: Bit) -> None:
        assert AndGate()(a, b) == expected

    def test_transistor_count(self) -> None:
        assert AndGate().transistor_count == 6

    def test_repr(self) -> None:
        assert str(AndGate()) == "AndGate(transistors=6, memory_bits=0)"


class TestOrGate:
    @pytest.mark.parametrize(
        ("a", "b", "expected"),
        [
            (0, 0, 0),
            (0, 1, 1),
            (1, 0, 1),
            (1, 1, 1),
        ],
    )
    def test_truth_table(self, a: Bit, b: Bit, expected: Bit) -> None:
        assert OrGate()(a, b) == expected

    def test_transistor_count(self) -> None:
        assert OrGate().transistor_count == 6

    def test_repr(self) -> None:
        assert str(OrGate()) == "OrGate(transistors=6, memory_bits=0)"

    def test_invalid_input(self) -> None:
        with pytest.raises(InvalidSignalError):
            OrGate()(INVALID_BIT, 0)


class TestXorGate:
    @pytest.mark.parametrize(
        ("a", "b", "expected"),
        [
            (0, 0, 0),
            (0, 1, 1),
            (1, 0, 1),
            (1, 1, 0),
        ],
    )
    def test_truth_table(self, a: Bit, b: Bit, expected: Bit) -> None:
        assert XorGate()(a, b) == expected

    def test_transistor_count(self) -> None:
        assert XorGate().transistor_count == 16

    def test_repr(self) -> None:
        assert str(XorGate()) == "XorGate(transistors=16, memory_bits=0)"


class TestXnorGate:
    @pytest.mark.parametrize(
        ("a", "b", "expected"),
        [
            (0, 0, 1),
            (0, 1, 0),
            (1, 0, 0),
            (1, 1, 1),
        ],
    )
    def test_truth_table(self, a: Bit, b: Bit, expected: Bit) -> None:
        assert XnorGate()(a, b) == expected

    def test_transistor_count(self) -> None:
        assert XnorGate().transistor_count == 18

    def test_repr(self) -> None:
        assert str(XnorGate()) == "XnorGate(transistors=18, memory_bits=0)"
