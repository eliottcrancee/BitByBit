"""
hardware.py
===========

Hardware simulation primitives.

This module is the lowest layer of the project. It knows neither CPU,
instruction nor assembler. It only provides the abstractions needed to
build things up tier by tier:

    transistors
        |
        v
    logic gates
        |
        v
    multiplexers / adders
        |
        v
    registers / memory
        |
        v
    ALU
        |
        v
    CPU

Logical model
-------------

    0       LOW / GND
    1       HIGH / VCC
    None    Z / high impedance

The model is deliberately numeric and pedagogical. It does not try to
simulate real voltages, currents, capacitances or physical latencies.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass
from typing import (
    Final,
    Literal,
    TypeVar,
)

# ============================================================================
# Fundamental types
# ============================================================================

Bit = Literal[0, 1]
Signal = Bit | None

BITS_8: Final[int] = 8

Byte = tuple[Bit, ...]
Bus = tuple[Signal, ...]
Bus8 = Bus  # Alias for clarity: a bus of 8 bits is a byte

EMPTY_BUS: Final[Bus8] = (None, None, None, None, None, None, None, None)


# ============================================================================
# Electrical constants
# ============================================================================

LOW: Final[Bit] = 0
HIGH: Final[Bit] = 1

GND: Final[Bit] = LOW
VCC: Final[Bit] = HIGH

HIGH_IMPEDANCE: Final[None] = None


# ============================================================================
# Exceptions
# ============================================================================


class HardwareError(Exception):
    """Generic hardware simulator error."""


class InvalidBitError(HardwareError):
    """A bit is not a valid logic value."""


class InvalidSignalError(HardwareError):
    """A signal is not a valid logic value."""


class BusConflictError(HardwareError):
    """Several drivers enforce conflicting values on a bus."""


class UnstableCircuitError(HardwareError):
    """A combinatorial feedback circuit did not reach a stable state."""


# ============================================================================
# Physical metadata
# ============================================================================


@dataclass(frozen=True, slots=True)
class HardwareCost:
    """
    Approximate hardware cost of a component.

    `transistors`
        Number of transistors required by the implementation.

    `memory_bits`
        Number of physical storage bits.

    The cost is intentionally simple. It may be refined later with area,
    power, gates, etc.
    """

    transistors: int = 0
    memory_bits: int = 0

    def __post_init__(self) -> None:
        if self.transistors < 0:
            raise ValueError("The number of transistors cannot be negative.")

        if self.memory_bits < 0:
            raise ValueError("The number of memory bits cannot be negative.")

    def __add__(self, other: HardwareCost) -> HardwareCost:
        return HardwareCost(
            transistors=self.transistors + other.transistors,
            memory_bits=self.memory_bits + other.memory_bits,
        )


# ============================================================================
# Base component
# ============================================================================


class Component:
    """
    Base class for every hardware component.

    A component can be:

    - elementary: a transistor, etc.
    - composite: built from other components.

    A component does not necessarily hold state. Sequential components
    manage their state in dedicated classes.

    The cost of a component is *derived* from its attributes. Every
    ``Component`` instance stored on the component, directly or inside
    a list or tuple attribute, is counted as a sub-component, and the
    counts are summed recursively. Composite components therefore
    declare nothing at all: a gate made of transistors automatically
    reports the sum of its transistors.

    Leaf components declare their own cost with the ``transistors``
    and ``memory_bits`` class attributes.

    ``memory_bits`` counts physical storage cells. It is declared on
    the smallest storage primitive, the SR latch. A master-slave flip
    flop is made of two latches, so it counts two physical bits.
    """

    # Transistors needed by this component alone, excluding any
    # sub-components. Only leaf components override this.
    transistors: int = 0

    # Physical storage bits provided by this component alone, excluding
    # any sub-components. Only the elementary storage element, the SR
    # latch, overrides this.
    memory_bits: int = 0

    def sub_components(self) -> Iterator[Component]:
        """Yield every component stored in this component's attributes."""
        for value in vars(self).values():
            if isinstance(value, Component):
                yield value
            elif isinstance(value, (list, tuple)):
                for item in value:
                    if isinstance(item, Component):
                        yield item

    @property
    def transistor_count(self) -> int:
        """Total number of transistors, sub-components included."""
        return self.transistors + sum(
            sub_component.transistor_count for sub_component in self.sub_components()
        )

    @property
    def bit_count(self) -> int:
        """Total number of physical storage bits, sub-components included."""
        return self.memory_bits + sum(
            sub_component.bit_count for sub_component in self.sub_components()
        )

    @property
    def cost(self) -> HardwareCost:
        """Aggregate hardware cost, sub-components included."""
        return HardwareCost(
            transistors=self.transistor_count,
            memory_bits=self.bit_count,
        )

    def debug(self) -> str:
        """Return a representation convenient for debugging."""
        return str(self)

    def __str__(self) -> str:
        return (
            f"{type(self).__name__}"
            f"(transistors={self.transistor_count}, "
            f"memory_bits={self.bit_count})"
        )


# ============================================================================
# Signal validation
# ============================================================================


def is_bit(value: object) -> bool:
    """Return True if `value` is exactly 0 or 1."""
    return value == LOW or value == HIGH


def validate_bit(value: object) -> Bit:
    """Validate and return a bit (0 or 1)."""
    if value == LOW:
        return LOW
    if value == HIGH:
        return HIGH
    raise InvalidSignalError(
        f"Invalid logic value: {value!r}. A bit must be either LOW (0) or HIGH (1)."
    )


def validate_signal(value: object) -> Signal:
    """Validate and return a signal, possibly in high impedance."""
    if value is None:
        return HIGH_IMPEDANCE

    return validate_bit(value)


# ============================================================================
# Transistors
# ============================================================================


class PMOS(Component):
    """
    Ideal PMOS transistor.

    In this model:

        gate = LOW  -> transistor conducting
        gate = HIGH -> transistor blocked

    When blocked, the output is in high impedance.
    """

    transistors: int = 1

    def __call__(self, gate: Signal, source: Signal) -> Signal:
        if gate is LOW:
            return source

        return HIGH_IMPEDANCE


class NMOS(Component):
    """
    Ideal NMOS transistor.

    In this model:

        gate = HIGH -> transistor conducting
        gate = LOW  -> transistor blocked

    When blocked, the output is in high impedance.
    """

    transistors: int = 1

    def __call__(self, gate: Signal, source: Signal) -> Signal:
        if gate is HIGH:
            return source

        return HIGH_IMPEDANCE


# ============================================================================
# Wires and buses
# ============================================================================


def wire(*signals: Signal) -> Signal:
    """
    Resolve several drivers connected to the same wire.

    Rules:

        wire()              -> Z
        wire(None, None)    -> Z
        wire(0, None)       -> 0
        wire(1, None)       -> 1
        wire(0, 0)          -> 0
        wire(1, 1)          -> 1
        wire(0, 1)          -> error

    A physical wire must not have two active drivers imposing different
    values at the same time.
    """

    result: Signal = HIGH_IMPEDANCE
    for signal in signals:
        if signal is None:
            continue
        if result is None:
            result = signal
        elif result != signal:
            raise BusConflictError(
                f"Electrical conflict: {result} and {signal} on the same wire."
            )
    return result


def bus8(*buses: Bus) -> Bus:
    """
    Resolve a set of drivers on an 8-bit bus.

    Example:

        bus8(
            (1, None, None, 0, None, None, None, None),
            (None, 0, None, None, None, None, 1, None),
        )

    produces:

        (1, 0, None, 0, None, None, 1, None)
    """
    if not buses:
        return EMPTY_BUS
    for b in buses:
        if len(b) != 8:
            raise ValueError(f"A bus must contain 8 bits, but {len(b)} were provided.")
    return tuple(wire(*row) for row in zip(*buses))


# ============================================================================
# Bus tools
# ============================================================================


def bits_to_int(
    bits: Sequence[Bit],
    *,
    msb_first: bool = False,
) -> int:
    """
    Convert a sequence of bits to an integer.

    By default, the first element is considered the least significant bit.

    Example:

        bits_to_int((1, 0, 1))
        -> 5
    """

    if not bits:
        return 0

    result = 0

    if msb_first:
        for bit in bits:
            result = (result << 1) | bit
    else:
        for index, bit in enumerate(bits):
            result |= bit << index

    return result


def int_to_bits(
    value: int,
    width: int,
    *,
    msb_first: bool = False,
) -> tuple[Bit, ...]:
    """
    Convert an integer to a sequence of bits.

    By default, the first returned bit is the least significant bit.

    Example:

        int_to_bits(5, 4)
        -> (1, 0, 1, 0)
    """

    if width <= 0:
        raise ValueError("The width must be strictly positive.")

    if value < 0:
        raise ValueError("The value cannot be negative.")

    if value >= (1 << width):
        raise ValueError(f"{value} does not fit in {width} bits.")

    resolved: list[Bit] = [
        HIGH if (value >> index) & 1 else LOW for index in range(width)
    ]

    if msb_first:
        return tuple(reversed(resolved))

    return tuple(resolved)


# ============================================================================
# Circuit stabilization
# ============================================================================

State = TypeVar("State")


def stabilize[State](
    get_state: Callable[[], State],
    update_state: Callable[[], None],
    max_iterations: int = 10,
) -> bool:
    """
    Let a circuit evolve until it reaches a stable state.

    This function is mainly intended for feedback circuits, e.g. latches
    and some memory cells.

    Returns True if a stable state is reached.

    Raises UnstableCircuitError if the circuit does not stabilize.
    """

    if max_iterations <= 0:
        raise ValueError("max_iterations must be strictly positive.")

    previous = get_state()

    for _ in range(max_iterations):
        update_state()
        current = get_state()

        if current == previous:
            return True

        previous = current

    raise UnstableCircuitError(
        f"The circuit did not stabilize after {max_iterations} iterations."
    )
