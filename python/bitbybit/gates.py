"""
gates.py
========

Logic gates implemented as ``Component`` objects on top of the transistor
primitives described in ``bitbybit.hardware``.

The lowest layer (``NotGate``, ``NandGate``, ``NorGate``) is wired at the
transistor level.  The next gates (``AndGate``, ``OrGate``, ``XorGate``,
``XnorGate``) are layered on top of them.

Numeric signal model inherited from ``hardware``:

    0     LOW / GND
    1     HIGH / VCC
    None  Z / high impedance
"""

from __future__ import annotations

from functools import cache

from bitbybit import FAST
from bitbybit.hardware import (
    GND,
    HIGH,
    LOW,
    NMOS,
    PMOS,
    VCC,
    Bit,
    Component,
    InvalidBitError,
    validate_bit,
    wire,
)


class NotGate(Component):
    """NOT gate (inverter), built with one PMOS and one NMOS."""

    def __init__(self) -> None:
        self.pmos = PMOS()
        self.nmos = NMOS()

    @staticmethod
    @cache
    def _fast_call(a: Bit) -> Bit:
        """Do the logic fast."""
        if a == HIGH:
            return LOW
        elif a == LOW:
            return HIGH
        else:
            raise InvalidBitError(f"Invalid bit value: {a}")

    def __call__(self, a: Bit) -> Bit:
        """Return the logical negation of ``a``.

        If ``a`` is LOW the PMOS drives VCC to the output.  If ``a`` is
        HIGH the NMOS drives GND to the output.  The output is the wire
        joining both transistor branches.
        """
        if FAST:
            return self._fast_call(a)
        return validate_bit(
            wire(
                self.pmos(gate=a, source=VCC),
                self.nmos(gate=a, source=GND),
            )
        )


class NandGate(Component):
    """Two-input NAND gate: 2 PMOS in parallel, 2 NMOS in series."""

    def __init__(self) -> None:
        self.pmos1 = PMOS()
        self.pmos2 = PMOS()
        self.nmos1 = NMOS()
        self.nmos2 = NMOS()

    @staticmethod
    @cache
    def _fast_call(a: Bit, b: Bit) -> Bit:
        """Do the logic fast."""
        if a == HIGH and b == HIGH:
            return LOW
        elif a == LOW or b == LOW:
            return HIGH
        else:
            raise InvalidBitError(f"Invalid bit values: {a}, {b}")

    def __call__(self, a: Bit, b: Bit) -> Bit:
        """Return HIGH except when A and B are both HIGH."""
        if FAST:
            return self._fast_call(a, b)
        pmos_output1 = self.pmos1(gate=a, source=VCC)
        pmos_output2 = self.pmos2(gate=b, source=VCC)

        nmos_intermediate = self.nmos1(gate=a, source=GND)
        nmos_output = self.nmos2(gate=b, source=nmos_intermediate)

        return validate_bit(wire(pmos_output1, pmos_output2, nmos_output))


class NorGate(Component):
    """Two-input NOR gate: 2 PMOS in series, 2 NMOS in parallel."""

    def __init__(self) -> None:
        self.pmos1 = PMOS()
        self.pmos2 = PMOS()
        self.nmos1 = NMOS()
        self.nmos2 = NMOS()

    @staticmethod
    @cache
    def _fast_call(a: Bit, b: Bit) -> Bit:
        """Do the logic fast."""
        if a == LOW and b == LOW:
            return HIGH
        elif a == HIGH or b == HIGH:
            return LOW
        else:
            raise InvalidBitError(f"Invalid bit values: {a}, {b}")

    def __call__(self, a: Bit, b: Bit) -> Bit:
        """Return HIGH only when both are LOW."""
        if FAST:
            return self._fast_call(a, b)
        pmos_intermediate = self.pmos1(gate=a, source=VCC)
        pmos_output = self.pmos2(gate=b, source=pmos_intermediate)

        nmos_output1 = self.nmos1(gate=a, source=GND)
        nmos_output2 = self.nmos2(gate=b, source=GND)

        return validate_bit(wire(pmos_output, nmos_output1, nmos_output2))


class AndGate(Component):
    """Two-input AND gate, composed of a NAND gate and a NOT gate."""

    def __init__(self) -> None:
        self.nand_gate = NandGate()
        self.not_gate = NotGate()

    @staticmethod
    @cache
    def _fast_call(a: Bit, b: Bit) -> Bit:
        """Do the logic fast."""
        if a == HIGH and b == HIGH:
            return HIGH
        elif a == LOW or b == LOW:
            return LOW
        else:
            raise InvalidBitError(f"Invalid bit values: {a}, {b}")

    def __call__(self, a: Bit, b: Bit) -> Bit:
        if FAST:
            return self._fast_call(a, b)
        return self.not_gate(self.nand_gate(a, b))


class OrGate(Component):
    """Two-input OR gate, composed of a NOR gate and a NOT gate."""

    def __init__(self) -> None:
        self.nor_gate = NorGate()
        self.not_gate = NotGate()

    @staticmethod
    @cache
    def _fast_call(a: Bit, b: Bit) -> Bit:
        """Do the logic fast."""
        if a == LOW and b == LOW:
            return LOW
        elif a == HIGH or b == HIGH:
            return HIGH
        else:
            raise InvalidBitError(f"Invalid bit values: {a}, {b}")

    def __call__(self, a: Bit, b: Bit) -> Bit:
        if FAST:
            return self._fast_call(a, b)
        return self.not_gate(self.nor_gate(a, b))


class XorGate(Component):
    """Two-input XOR gate, composed of four NAND gates."""

    def __init__(self) -> None:
        self.nand1 = NandGate()
        self.nand2 = NandGate()
        self.nand3 = NandGate()
        self.nand4 = NandGate()

    @staticmethod
    @cache
    def _fast_call(a: Bit, b: Bit) -> Bit:
        """Do the logic fast."""
        if a == LOW and b == LOW or a == HIGH and b == HIGH:
            return LOW
        elif (a == HIGH and b == LOW) or (a == LOW and b == HIGH):
            return HIGH
        else:
            raise InvalidBitError(f"Invalid bit values: {a}, {b}")

    def __call__(self, a: Bit, b: Bit) -> Bit:
        if FAST:
            return self._fast_call(a, b)
        nand_ab = self.nand1(a, b)
        nand_a = self.nand2(a, nand_ab)
        nand_b = self.nand3(b, nand_ab)
        return self.nand4(nand_a, nand_b)


class XnorGate(Component):
    """Two-input XNOR gate, composed of a XOR gate and a NOT gate."""

    def __init__(self) -> None:
        self.xor_gate = XorGate()
        self.not_gate = NotGate()

    @staticmethod
    @cache
    def _fast_call(a: Bit, b: Bit) -> Bit:
        """Do the logic fast."""
        if a == LOW and b == LOW or a == HIGH and b == HIGH:
            return HIGH
        elif (a == HIGH and b == LOW) or (a == LOW and b == HIGH):
            return LOW
        else:
            raise InvalidBitError(f"Invalid bit values: {a}, {b}")

    def __call__(self, a: Bit, b: Bit) -> Bit:
        if FAST:
            return self._fast_call(a, b)
        return self.not_gate(self.xor_gate(a, b))
