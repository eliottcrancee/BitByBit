"""
decoder.py
==========

Binary decoders built on top of the logic gates from ``pytranscpu.gates``.

A decoder activates exactly one output line for a given binary input
value. For ``Decoder2to4``, the output index activated by inputs
``(a, b)`` is ``2 * a + b``, meaning ``a`` is the most significant bit
of the pair.

``Decoder4to16`` cascades two ``Decoder2to4`` and ANDs their outputs.
Its 4-bit input is given least significant bit first, so the activated
output index is ``inputs[0] + 2 * inputs[1] + 4 * inputs[2] +
8 * inputs[3]``.
"""

from __future__ import annotations

from pytranscpu.gates import AndGate, NotGate
from pytranscpu.hardware import Bit, Component


class Decoder2to4(Component):
    """Two-to-four decoder: 2 NOT, 4 AND."""

    def __init__(self) -> None:
        self.not_a = NotGate()
        self.not_b = NotGate()
        self.and_gates = [AndGate() for _ in range(4)]

    def __call__(self, a: Bit, b: Bit) -> tuple[Bit, Bit, Bit, Bit]:
        """Activate output ``2 * a + b`` and keep the others LOW."""
        not_a = self.not_a(a)
        not_b = self.not_b(b)

        return (
            self.and_gates[0](not_a, not_b),
            self.and_gates[1](not_a, b),
            self.and_gates[2](a, not_b),
            self.and_gates[3](a, b),
        )


class Decoder4to16(Component):
    """Four-to-sixteen decoder: two 2-to-4 decoders and 16 AND gates."""

    def __init__(self) -> None:
        self.low_decoder = Decoder2to4()
        self.high_decoder = Decoder2to4()
        self.and_gates = [AndGate() for _ in range(16)]

    def __call__(self, inputs: tuple[Bit, Bit, Bit, Bit]) -> tuple[Bit, ...]:
        """Activate the output matching the LSB-first value of ``inputs``."""
        if len(inputs) != 4:
            raise ValueError("The input signal must contain exactly 4 bits.")

        # ``Decoder2to4(a, b)`` activates index ``2 * a + b``, so the most
        # significant bit of each pair must be given first.
        high_group = self.high_decoder(inputs[3], inputs[2])
        low_group = self.low_decoder(inputs[1], inputs[0])

        return tuple(
            self.and_gates[index](high_group[index // 4], low_group[index % 4])
            for index in range(16)
        )
