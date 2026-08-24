"""
test_decoder.py
===============

Tests for the decoders in ``pytranscpu/decoder.py``.
"""

from __future__ import annotations

from typing import cast

import pytest

from pytranscpu.decoder import Decoder2to4, Decoder4to16
from pytranscpu.hardware import Bit, int_to_bits


class TestDecoder2to4:
    @pytest.mark.parametrize(
        ("a", "b", "expected"),
        [
            (0, 0, (1, 0, 0, 0)),
            (0, 1, (0, 1, 0, 0)),
            (1, 0, (0, 0, 1, 0)),
            (1, 1, (0, 0, 0, 1)),
        ],
    )
    def test_truth_table(self, a: Bit, b: Bit, expected: tuple[Bit, ...]) -> None:
        assert Decoder2to4()(a, b) == expected

    def test_transistor_count(self) -> None:
        assert Decoder2to4().transistor_count == 28

    def test_repr(self) -> None:
        assert str(Decoder2to4()) == "Decoder2to4(transistors=28, memory_bits=0)"


class TestDecoder4to16:
    @pytest.mark.parametrize("value", range(16))
    def test_activates_exactly_one_line(self, value: int) -> None:
        decoder = Decoder4to16()
        inputs = cast(tuple[Bit, Bit, Bit, Bit], int_to_bits(value, 4))

        outputs = decoder(inputs)

        assert outputs[value] == 1
        assert sum(outputs) == 1

    def test_rejects_inputs_not_4_bits_long(self) -> None:
        decoder = Decoder4to16()

        with pytest.raises(ValueError):
            decoder(cast(tuple[Bit, Bit, Bit, Bit], (0, 0, 0)))

        with pytest.raises(ValueError):
            decoder(cast(tuple[Bit, Bit, Bit, Bit], (0, 0, 0, 0, 0)))

    def test_transistor_count(self) -> None:
        assert Decoder4to16().transistor_count == 152

    def test_repr(self) -> None:
        assert str(Decoder4to16()) == "Decoder4to16(transistors=152, memory_bits=0)"
