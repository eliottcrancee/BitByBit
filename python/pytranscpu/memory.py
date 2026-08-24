"""
memory.py
=========

Storage components layered on top of ``pytranscpu.latches``,
``pytranscpu.arithmetic``, ``pytranscpu.mux`` and
``pytranscpu.decoder``.

All multi-bit values are LSB-first tuples, matching the convention of
``pytranscpu.hardware``.

    Register8Bits
        Eight D flip-flops with save/load. Writes on the rising edge
        of ``clock`` when ``save`` is HIGH; outputs float when ``load``
        is LOW.

    ProgramCounter4Bits
        Increments itself on each rising clock edge, or loads ``data``
        instead when ``save`` is HIGH. Only the four low bits are
        exposed; the value wraps back to zero on overflow.

    Ram256Bits
        Sixteen 8-bit registers addressed by a 4-to-16 decoder. Only
        the addressed register drives the output bus on load.
"""

from __future__ import annotations

from typing import cast

from pytranscpu.arithmetic import Adder8Bits
from pytranscpu.decoder import Decoder4to16
from pytranscpu.gates import AndGate
from pytranscpu.hardware import (
    BITS_8,
    HIGH,
    LOW,
    NMOS,
    Bit,
    Bus8,
    Byte,
    Component,
    Signal,
    bus8,
)
from pytranscpu.latches import DFlipFlopSaveLoad
from pytranscpu.mux import Mux8bits2x1

ADDRESS_BITS: int = 4
RAM_REGISTERS: int = 16

ZERO_BYTE: Byte = (LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW)
ONE_BYTE: Byte = (HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW)


class Register8Bits(Component):
    """Eight-bit register: writes on the rising edge when ``save`` is HIGH."""

    def __init__(self) -> None:
        self.flip_flops = [DFlipFlopSaveLoad() for _ in range(BITS_8)]

    @property
    def state(self) -> Byte:
        """The stored byte, LSB first."""
        return tuple(flip_flop.q for flip_flop in self.flip_flops)

    def __call__(self, data: Byte, clock: Bit, save: Bit, load: Bit) -> Bus8:
        """Store ``data`` on the rising edge and return the gated outputs."""
        return cast(
            Bus8,
            tuple(
                self.flip_flops[index](data[index], clock, save, load)[0]
                for index in range(BITS_8)
            ),
        )


class ProgramCounter4Bits(Component):
    """Four-bit program counter: increments, or loads on rising edge."""

    def __init__(self) -> None:
        self.register = Register8Bits()
        self.adder = Adder8Bits()
        self.load_mux = Mux8bits2x1()
        self.overflow_mux = Mux8bits2x1()
        self.output_nmos = [NMOS() for _ in range(ADDRESS_BITS)]

    @property
    def state(self) -> Byte:
        """The full byte stored by the inner register."""
        return self.register.state

    def __call__(
        self, data: tuple[Bit, Bit, Bit, Bit], clock: Bit, save: Bit, load: Bit
    ) -> tuple[Signal, ...]:
        """Increment, or load ``data`` when ``save`` is HIGH.

        The next value is computed combinationally from the stored
        one: ``state + 1`` is selected against ``data`` by ``save``,
        and the result wraps to zero when the fifth bit overflows. The
        value is written on the rising edge, then the four low bits
        are gated by ``load``.

        The feedback path goes through the register's flip-flops, so a
        single propagation pass computes the new state.
        """
        incremented, _ = self.adder(self.register.state, ONE_BYTE)
        selected = self.load_mux(
            incremented, data + (LOW, LOW, LOW, LOW), save
        )
        wrapped = self.overflow_mux(selected, ZERO_BYTE, selected[4])
        stored = self.register(wrapped, clock, clock, HIGH)

        return tuple(
            self.output_nmos[index](load, stored[index])
            for index in range(ADDRESS_BITS)
        )


class Ram256Bits(Component):
    """256-bit RAM: 16 registers of 8 bits selected by a 4-bit address."""

    def __init__(self) -> None:
        self.registers = [Register8Bits() for _ in range(RAM_REGISTERS)]
        self.decoder = Decoder4to16()
        self.save_gates = [AndGate() for _ in range(RAM_REGISTERS)]
        self.load_gates = [AndGate() for _ in range(RAM_REGISTERS)]

    @property
    def state(self) -> tuple[Byte, ...]:
        """The stored content of every register, in address order."""
        return tuple(register.state for register in self.registers)

    def __call__(
        self,
        data: Byte,
        address: tuple[Bit, Bit, Bit, Bit],
        clock: Bit,
        save: Bit,
        load: Bit,
    ) -> Bus8:
        """Write ``data`` at ``address`` on the rising edge, or read it.

        The decoder activates a single register. The ``save`` and
        ``load`` signals are ANDed with that selection, so only the
        addressed register can be written or can drive the output bus.
        """
        selected = self.decoder(address)

        outputs = [
            self.registers[index](
                data,
                clock,
                self.save_gates[index](selected[index], save),
                self.load_gates[index](selected[index], load),
            )
            for index in range(RAM_REGISTERS)
        ]

        return bus8(*outputs)
