"""
test_hardware.py
================

Tests for the hardware simulation primitives in ``pytranscpu/hardware.py``.
"""

from __future__ import annotations

import pytest

from pytranscpu.hardware import (
    GND,
    HIGH,
    HIGH_IMPEDANCE,
    LOW,
    NMOS,
    PMOS,
    VCC,
    BusConflictError,
    InvalidSignalError,
    UnstableCircuitError,
    bits_to_int,
    bus8,
    int_to_bits,
    is_bit,
    stabilize,
    validate_bit,
    validate_signal,
    wire,
)

# ----------------------------------------------------------------------------
# Transistors
# ----------------------------------------------------------------------------


class TestTransistors:
    def test_transistor_count(self) -> None:
        assert PMOS().transistor_count == 1
        assert NMOS().transistor_count == 1

    def test_pmos(self) -> None:
        pmos = PMOS()
        assert pmos(LOW, VCC) == VCC
        assert pmos(HIGH, VCC) is None

    def test_nmos(self) -> None:
        nmos = NMOS()
        assert nmos(HIGH, GND) == GND
        assert nmos(LOW, GND) is None


# ----------------------------------------------------------------------------
# Signal validation
# ----------------------------------------------------------------------------


class TestValidation:
    def test_is_bit_true(self) -> None:
        assert is_bit(LOW) is True
        assert is_bit(HIGH) is True

    def test_is_bit_false(self) -> None:
        assert is_bit(None) is False
        assert is_bit(2) is False
        assert is_bit("1") is False

    def test_validate_bit_valid(self) -> None:
        assert validate_bit(LOW) == LOW
        assert validate_bit(HIGH) == HIGH

    def test_validate_bit_invalid(self) -> None:
        with pytest.raises(InvalidSignalError):
            validate_bit(2)

    def test_validate_signal_none(self) -> None:
        assert validate_signal(None) is HIGH_IMPEDANCE

    def test_validate_signal_valid(self) -> None:
        assert validate_signal(LOW) == LOW
        assert validate_signal(HIGH) == HIGH


# ----------------------------------------------------------------------------
# Wire
# ----------------------------------------------------------------------------


class TestWire:
    def test_empty_returns_high_impedance(self) -> None:
        assert wire() is None

    def test_all_high_impedance(self) -> None:
        assert wire(None, None) is None

    def test_single_driver(self) -> None:
        assert wire(HIGH, None) == HIGH
        assert wire(LOW, None) == LOW

    def test_consistent_drivers(self) -> None:
        assert wire(HIGH, HIGH) == HIGH
        assert wire(LOW, LOW) == LOW

    def test_conflict(self) -> None:
        with pytest.raises(BusConflictError):
            wire(HIGH, LOW)


# ----------------------------------------------------------------------------
# Bus
# ----------------------------------------------------------------------------


class TestBus8:
    def test_resolves_a_bus(self) -> None:
        result = bus8(
            (None, None, HIGH, None, None, LOW, None, LOW),
            (LOW, HIGH, None, HIGH, LOW, None, HIGH, None),
            (None, None, None, None, None, None, None, None),
        )

        assert result == (
            LOW,
            HIGH,
            HIGH,
            HIGH,
            LOW,
            LOW,
            HIGH,
            LOW,
        )


# ----------------------------------------------------------------------------
# Bit / integer conversion
# ----------------------------------------------------------------------------


class TestBitIntegerConversion:
    def test_bits_to_int(self) -> None:
        assert bits_to_int((1, 0, 1)) == 5

    def test_bits_to_int_msb_first(self) -> None:
        assert bits_to_int((1, 0, 1), msb_first=True) == 5

    def test_int_to_bits(self) -> None:
        assert int_to_bits(5, 4) == (1, 0, 1, 0)

    def test_int_to_bits_msb_first(self) -> None:
        assert int_to_bits(5, 4, msb_first=True) == (0, 1, 0, 1)

    def test_int_to_bits_rejects_width(self) -> None:
        with pytest.raises(ValueError):
            int_to_bits(5, 0)

    def test_int_to_bits_rejects_negative(self) -> None:
        with pytest.raises(ValueError):
            int_to_bits(-1, 4)

    def test_int_to_bits_rejects_overflow(self) -> None:
        with pytest.raises(ValueError):
            int_to_bits(16, 4)


# ----------------------------------------------------------------------------
# Stabilization
# ----------------------------------------------------------------------------


class TestStabilize:
    def test_unstable_when_budget_exhausted(self) -> None:
        state = {"value": 0}

        def get_state() -> int:
            return state["value"]

        def update_state() -> None:
            state["value"] += 1

        # With a low enough iteration budget the circuit never stabilizes.
        with pytest.raises(UnstableCircuitError):
            stabilize(get_state, update_state, max_iterations=3)

    def test_stabilizes_once_state_is_frozen(self) -> None:
        state = {"value": 0}
        frozen = False

        def get_state() -> int:
            return state["value"]

        def update_state() -> None:
            nonlocal frozen
            if not frozen:
                state["value"] += 1
                frozen = True

        assert stabilize(get_state, update_state, max_iterations=10) is True

    def test_rejects_non_positive_max_iterations(self) -> None:
        with pytest.raises(ValueError):
            stabilize(lambda: 0, lambda: None, max_iterations=0)
