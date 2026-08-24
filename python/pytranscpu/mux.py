"""
mux.py
======

Multiplexers built on top of the logic gates from ``pytranscpu.gates``.

A 2-to-1 multiplexer forwards input ``a`` when ``select`` is LOW and
input ``b`` when ``select`` is HIGH:

    output = (a AND NOT select) OR (b AND select)

The 8-bit variant simply places eight of them in parallel.
"""

from __future__ import annotations

from pytranscpu.gates import AndGate, NotGate, OrGate
from pytranscpu.hardware import BITS_8, Bit, Byte, Component


class Mux2x1(Component):
    """Two-to-one multiplexer: 1 NOT, 2 AND, 1 OR."""

    def __init__(self) -> None:
        self.not_select = NotGate()
        self.and_a = AndGate()
        self.and_b = AndGate()
        self.or_output = OrGate()

    def __call__(self, a: Bit, b: Bit, select: Bit) -> Bit:
        """Return ``a`` if ``select`` is LOW, otherwise ``b``."""
        not_select = self.not_select(select)
        path_a = self.and_a(a, not_select)
        path_b = self.and_b(b, select)
        return self.or_output(path_a, path_b)


class Mux8bits2x1(Component):
    """Eight two-to-one multiplexers in parallel, one per bit."""

    def __init__(self) -> None:
        self.muxes = [Mux2x1() for _ in range(BITS_8)]

    def __call__(self, a: Byte, b: Byte, select: Bit) -> Byte:
        """Return ``a`` if ``select`` is LOW, otherwise ``b``."""
        return tuple(
            self.muxes[index](a[index], b[index], select) for index in range(BITS_8)
        )
