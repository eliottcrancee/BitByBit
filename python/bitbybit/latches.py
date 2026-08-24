"""
latches.py
==========

Sequential circuits built on top of the gates from ``bitbybit.gates``
and the multiplexer from ``bitbybit.mux``.

Components, from the simplest to the most elaborate:

    SRLatch
        Two cross-coupled NOR gates. Stores one bit. SET and RESET
        both HIGH is forbidden.

    DLatch
        An SR latch fed by ``data`` and ``NOT data``, gated by
        ``enable``. Transparent while ``enable`` is HIGH.

    DFlipFlop
        Master-slave pair of D latches. Updates ``q`` only on the
        rising edge of ``clock``.

    DFlipFlopSave
        A D flip-flop whose write is gated by ``save``: on a rising
        edge, ``q`` keeps its previous value when ``save`` is LOW.

    DFlipFlopSaveLoad
        A D flip-flop with save, whose outputs are gated by ``load``.
        When ``load`` is LOW the outputs float in high impedance.

    OneHotCounter6Bits
        A 6-bit ring of D flip-flops. Exactly one bit is HIGH and it
        advances by one position on each rising clock edge.
"""

from __future__ import annotations

from typing import Final

from bitbybit import FAST
from bitbybit.gates import AndGate, NorGate, NotGate
from bitbybit.hardware import (
    HIGH,
    LOW,
    NMOS,
    Bit,
    Component,
    HardwareError,
    Signal,
    stabilize,
)
from bitbybit.mux import Mux2x1

COUNTER_BITS: Final[int] = 6


class SRLatch(Component):
    """SR latch made of two cross-coupled NOR gates."""

    # An SR latch is the elementary storage element: one physical bit.
    memory_bits: int = 1

    def __init__(self) -> None:
        self.nor1 = NorGate()
        self.nor2 = NorGate()
        self.q: Bit = LOW
        self.q_bar: Bit = HIGH

    def _fast_call(self, set_signal: Bit, reset_signal: Bit) -> tuple[Bit, Bit]:
        """Do the logic without using the stabilize() function."""
        if set_signal == HIGH:
            self.q = HIGH
            self.q_bar = LOW
        elif reset_signal == HIGH:
            self.q = LOW
            self.q_bar = HIGH

        return self.q, self.q_bar

    def __call__(self, set_signal: Bit, reset_signal: Bit) -> tuple[Bit, Bit]:
        """Update the latch and return ``(q, q_bar)``.

        SET=1/RESET=0 sets the latch, SET=0/RESET=1 resets it and
        SET=0/RESET=0 keeps the previous state.
        """
        # Not an electrical fault: each NOR gate would happily output
        # LOW, so the hardware layer sees nothing wrong. The combination
        # is rejected because Q and Q_bar would stop being
        # complementary, and real silicon becomes metastable when both
        # inputs are released together. The latch refuses to pretend it
        # can store a valid state in these conditions.
        if set_signal == HIGH and reset_signal == HIGH:
            raise HardwareError("SET and RESET cannot be HIGH at the same time.")

        if FAST:
            return self._fast_call(set_signal, reset_signal)

        def get_state() -> tuple[Bit, Bit]:
            return self.q, self.q_bar

        def update() -> None:
            self.q_bar = self.nor1(set_signal, self.q)
            self.q = self.nor2(reset_signal, self.q_bar)

        stabilize(get_state, update)

        return self.q, self.q_bar


class DLatch(Component):
    """D latch: transparent while ``enable`` is HIGH, holds otherwise."""

    def __init__(self) -> None:
        self.sr_latch = SRLatch()
        self.not_data = NotGate()
        self.and_set = AndGate()
        self.and_reset = AndGate()

    @property
    def q(self) -> Bit:
        return self.sr_latch.q

    @property
    def q_bar(self) -> Bit:
        return self.sr_latch.q_bar

    def __call__(self, data: Bit, enable: Bit) -> tuple[Bit, Bit]:
        """Follow ``data`` while ``enable`` is HIGH, hold otherwise."""
        set_signal = self.and_set(data, enable)
        reset_signal = self.and_reset(self.not_data(data), enable)
        return self.sr_latch(set_signal, reset_signal)


class DFlipFlop(Component):
    """Master-slave D flip-flop, updated on the rising edge of ``clock``."""

    def __init__(self) -> None:
        self.master_latch = DLatch()
        self.slave_latch = DLatch()
        self.not_clock = NotGate()

    @property
    def q(self) -> Bit:
        return self.slave_latch.q

    @property
    def q_bar(self) -> Bit:
        return self.slave_latch.q_bar

    def __call__(self, data: Bit, clock: Bit) -> tuple[Bit, Bit]:
        """Capture ``data`` on the rising edge of ``clock``."""
        master_output, _ = self.master_latch(data, self.not_clock(clock))
        return self.slave_latch(master_output, clock)


class DFlipFlopSave(Component):
    """D flip-flop whose writes are gated by ``save``."""

    def __init__(self) -> None:
        self.master_latch = DLatch()
        self.slave_latch = DLatch()
        self.not_clock = NotGate()
        self.mux = Mux2x1()

    @property
    def q(self) -> Bit:
        return self.slave_latch.q

    @property
    def q_bar(self) -> Bit:
        return self.slave_latch.q_bar

    def __call__(self, data: Bit, clock: Bit, save: Bit) -> tuple[Bit, Bit]:
        """Capture ``data`` on the rising edge only if ``save`` is HIGH.

        When ``save`` is LOW, the slave is fed its own output, so the
        rising edge keeps the previous value.
        """
        master_output, _ = self.master_latch(data, self.not_clock(clock))
        selected_input = self.mux(self.slave_latch.q, master_output, save)
        return self.slave_latch(selected_input, clock)


class DFlipFlopSaveLoad(Component):
    """D flip-flop with save, whose outputs are gated by ``load``."""

    def __init__(self) -> None:
        self.flip_flop = DFlipFlopSave()
        self.nmos_q = NMOS()
        self.nmos_q_bar = NMOS()

    @property
    def q(self) -> Bit:
        return self.flip_flop.q

    @property
    def q_bar(self) -> Bit:
        return self.flip_flop.q_bar

    def __call__(
        self, data: Bit, clock: Bit, save: Bit, load: Bit
    ) -> tuple[Signal, Signal]:
        """Behave as ``DFlipFlopSave``; float the outputs when ``load`` is LOW."""
        q, q_bar = self.flip_flop(data, clock, save)
        return self.nmos_q(load, q), self.nmos_q_bar(load, q_bar)


class OneHotCounter6Bits(Component):
    """Ring counter shifting a single HIGH bit on each rising clock edge."""

    def __init__(self, initial_position: int = COUNTER_BITS - 1) -> None:
        if not 0 <= initial_position < COUNTER_BITS:
            raise ValueError(
                f"initial_position must be within [0, {COUNTER_BITS - 1}]."
            )

        self.flip_flops = [DFlipFlop() for _ in range(COUNTER_BITS)]

        # Force the starting bit HIGH with a LOW->HIGH clock sequence.
        self.flip_flops[initial_position](HIGH, LOW)
        self.flip_flops[initial_position](HIGH, HIGH)

    @property
    def state(self) -> tuple[Bit, ...]:
        """The current position of the single HIGH bit."""
        return tuple(flip_flop.q for flip_flop in self.flip_flops)

    def __call__(self, clock: Bit) -> tuple[Bit, ...]:
        """Shift the HIGH bit by one position on the rising edge.

        The ring is wired like the real circuit: each flip-flop's data
        input is the previous flip-flop's output, and the last output
        is fed back to the first one. The edge-triggered flip-flops
        break the feedback loop, so a single propagation pass over the
        ring computes the new state.
        """
        self.flip_flops[0](self.flip_flops[-1].q, clock)

        for index in range(1, COUNTER_BITS):
            previous_q = self.flip_flops[index - 1].q
            self.flip_flops[index](previous_q, clock)

        return self.state
