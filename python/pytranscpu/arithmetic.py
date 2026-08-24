"""
arithmetic.py
=============

Binary adders and the arithmetic/logic unit of the SAP-1, layered on
top of the logic gates from ``pytranscpu.gates``.

Bits are given least significant bit first, both for the inputs and
the outputs:

    (1, 0, 1)  ->  5

The ALU performs additions and subtractions, and exposes carry and
zero flags. Its result bus is gated by a ``load`` signal: when
``load`` is LOW the outputs float in high impedance, so the bus can
be shared with other drivers.
"""

from __future__ import annotations

from typing import cast

from pytranscpu.gates import AndGate, NotGate, OrGate, XorGate
from pytranscpu.hardware import (
    BITS_8,
    LOW,
    NMOS,
    Bit,
    Bus8,
    Byte,
    Component,
)


class HalfAdder(Component):
    """Single-bit half adder: 1 XOR for the sum, 1 AND for the carry."""

    def __init__(self) -> None:
        self.xor_gate = XorGate()
        self.and_gate = AndGate()

    def __call__(self, a: Bit, b: Bit) -> tuple[Bit, Bit]:
        """Return ``(sum, carry_out)`` for the two input bits."""
        return self.xor_gate(a, b), self.and_gate(a, b)


class FullAdder(Component):
    """Single-bit full adder: two half adders and 1 OR."""

    def __init__(self) -> None:
        self.half_adder1 = HalfAdder()
        self.half_adder2 = HalfAdder()
        self.or_gate = OrGate()

    def __call__(self, a: Bit, b: Bit, carry_in: Bit) -> tuple[Bit, Bit]:
        """Return ``(sum, carry_out)`` for ``a + b + carry_in``."""
        partial_sum, partial_carry = self.half_adder1(a, b)
        sum_bit, carry_from_low = self.half_adder2(partial_sum, carry_in)
        return sum_bit, self.or_gate(partial_carry, carry_from_low)


class Adder8Bits(Component):
    """Eight-bit ripple-carry adder: eight full adders in cascade."""

    def __init__(self) -> None:
        self.full_adders = [FullAdder() for _ in range(BITS_8)]

    def __call__(self, a: Byte, b: Byte, carry_in: Bit = LOW) -> tuple[Byte, Bit]:
        """Return ``(sum, carry_out)`` for ``a + b + carry_in``."""
        sum_bits: list[Bit] = []
        carry: Bit = carry_in

        for index in range(BITS_8):
            sum_bit, carry = self.full_adders[index](a[index], b[index], carry)
            sum_bits.append(sum_bit)

        return tuple(sum_bits), carry


class ALU8Bits(Component):
    """
    The arithmetic/logic unit of the SAP-1.

    Computes ``a + b`` or ``a - b`` depending on the ``subtract``
    control signal, and exposes the carry and zero flags:

    ``carry_flag``
        The carry out of the most significant bit. On subtraction it
        follows the two's-complement convention: HIGH means no borrow
        occurred, i.e. ``a >= b``.

    ``zero_flag``
        HIGH when the result is zero.

    The subtraction is computed as ``a + NOT b + 1``: each bit of
    ``b`` goes through an XOR gate controlled by ``subtract``, and the
    adder's carry-in is ``subtract`` itself.

    The result bus is gated by ``load``: when ``load`` is LOW the
    outputs float in high impedance, so the bus can be shared with
    other drivers. The flags remain valid in both cases.
    """

    def __init__(self) -> None:
        self.adder = Adder8Bits()
        self.b_inverters = [XorGate() for _ in range(BITS_8)]
        self.zero_or_gates = [OrGate() for _ in range(BITS_8 - 1)]
        self.zero_not = NotGate()
        self.output_nmos = [NMOS() for _ in range(BITS_8)]

    def __call__(
        self, a: Byte, b: Byte, subtract: Bit, load: Bit
    ) -> tuple[Bus8, Bit, Bit]:
        """Return ``(result, carry_flag, zero_flag)`` for ``a + b`` or ``a - b``."""
        inverted_b = cast(
            Byte,
            tuple(
                self.b_inverters[index](b[index], subtract) for index in range(BITS_8)
            ),
        )
        sum_bits, carry_flag = self.adder(a, inverted_b, carry_in=subtract)
        zero_flag = self._zero_flag(sum_bits)

        result = cast(
            Bus8,
            tuple(
                self.output_nmos[index](load, sum_bits[index])
                for index in range(BITS_8)
            ),
        )

        return result, carry_flag, zero_flag

    def _zero_flag(self, bits: Byte) -> Bit:
        """Return HIGH when every bit of ``bits`` is LOW.

        Fixed wiring: a tree of OR gates detects whether at least one
        bit is HIGH, and the final NOT inverts the result.
        """
        pair_01 = self.zero_or_gates[0](bits[0], bits[1])
        pair_23 = self.zero_or_gates[1](bits[2], bits[3])
        pair_45 = self.zero_or_gates[2](bits[4], bits[5])
        pair_67 = self.zero_or_gates[3](bits[6], bits[7])

        half_0123 = self.zero_or_gates[4](pair_01, pair_23)
        half_4567 = self.zero_or_gates[5](pair_45, pair_67)

        any_bit_high = self.zero_or_gates[6](half_0123, half_4567)

        return self.zero_not(any_bit_high)
