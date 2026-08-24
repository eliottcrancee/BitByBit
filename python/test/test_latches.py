"""
test_latches.py
===============

Tests for the sequential circuits in ``pytranscpu/latches.py``.
"""

from __future__ import annotations

import pytest

from pytranscpu.hardware import HardwareError
from pytranscpu.latches import (
    COUNTER_BITS,
    DFlipFlop,
    DFlipFlopSave,
    DFlipFlopSaveLoad,
    DLatch,
    OneHotCounter6Bits,
    SRLatch,
)


class TestSRLatch:
    def test_initial_state_is_reset(self) -> None:
        assert SRLatch()(0, 0) == (0, 1)

    def test_set_then_hold(self) -> None:
        latch = SRLatch()
        assert latch(1, 0) == (1, 0)
        assert latch(0, 0) == (1, 0)

    def test_reset_then_hold(self) -> None:
        latch = SRLatch()
        latch(1, 0)
        assert latch(0, 1) == (0, 1)
        assert latch(0, 0) == (0, 1)

    def test_set_and_reset_high_is_forbidden(self) -> None:
        with pytest.raises(HardwareError):
            SRLatch()(1, 1)

    def test_transistor_count(self) -> None:
        assert SRLatch().transistor_count == 8
        assert SRLatch().bit_count == 1


class TestDLatch:
    def test_follows_data_while_enabled_and_holds(self) -> None:
        latch = DLatch()
        assert latch(0, 0) == (0, 1)
        assert latch(1, 0) == (0, 1)
        assert latch(0, 1) == (0, 1)
        assert latch(1, 1) == (1, 0)
        assert latch(0, 0) == (1, 0)
        assert latch(1, 0) == (1, 0)
        assert latch(1, 1) == (1, 0)
        assert latch(0, 1) == (0, 1)

    def test_transistor_count(self) -> None:
        assert DLatch().transistor_count == 22
        assert DLatch().bit_count == 1  # its inner SR latch


class TestDFlipFlop:
    def test_updates_only_on_rising_edge(self) -> None:
        flip_flop = DFlipFlop()
        assert flip_flop(0, 0) == (0, 1)
        assert flip_flop(1, 0) == (0, 1)  # data prepared while clock is LOW
        assert flip_flop(0, 0) == (0, 1)
        assert flip_flop(0, 1) == (0, 1)  # rising edge with data 0
        assert flip_flop(1, 1) == (0, 1)  # no edge, no update
        assert flip_flop(1, 0) == (0, 1)
        assert flip_flop(1, 1) == (1, 0)  # rising edge with data 1
        assert flip_flop(0, 1) == (1, 0)
        assert flip_flop(0, 0) == (1, 0)
        assert flip_flop(0, 1) == (0, 1)

    def test_transistor_count(self) -> None:
        assert DFlipFlop().transistor_count == 46
        # A master-slave flip-flop is made of two latches: two physical bits.
        assert DFlipFlop().bit_count == 2


class TestDFlipFlopSave:
    def test_save_low_never_writes(self) -> None:
        flip_flop = DFlipFlopSave()
        for data in (0, 1, 0, 0, 1, 1, 0, 0):
            for clock in (0, 1):
                assert flip_flop(data, clock, 0) == (0, 1)

    def test_save_high_behaves_like_a_flip_flop(self) -> None:
        flip_flop = DFlipFlopSave()
        assert flip_flop(0, 0, 1) == (0, 1)
        assert flip_flop(1, 0, 1) == (0, 1)
        assert flip_flop(0, 0, 1) == (0, 1)
        assert flip_flop(0, 1, 1) == (0, 1)
        assert flip_flop(1, 1, 1) == (0, 1)
        assert flip_flop(1, 0, 1) == (0, 1)
        assert flip_flop(1, 1, 1) == (1, 0)
        assert flip_flop(0, 1, 1) == (1, 0)
        assert flip_flop(0, 0, 1) == (1, 0)
        assert flip_flop(0, 1, 1) == (0, 1)

    def test_transistor_count(self) -> None:
        assert DFlipFlopSave().transistor_count == 66


class TestDFlipFlopSaveLoad:
    def test_load_low_floats_the_outputs(self) -> None:
        assert DFlipFlopSaveLoad()(0, 0, 0, 0) == (None, None)

    def test_load_high_exposes_the_state(self) -> None:
        flip_flop = DFlipFlopSaveLoad()
        assert flip_flop(1, 0, 1, 1) == (0, 1)
        assert flip_flop(1, 1, 1, 1) == (1, 0)

    def test_transistor_count(self) -> None:
        assert DFlipFlopSaveLoad().transistor_count == 68


class TestOneHotCounter6Bits:
    def test_initial_state(self) -> None:
        assert OneHotCounter6Bits().state == (0, 0, 0, 0, 0, 1)

    def test_custom_initial_position(self) -> None:
        assert OneHotCounter6Bits(initial_position=2).state == (0, 0, 1, 0, 0, 0)

    def test_rejects_invalid_initial_position(self) -> None:
        with pytest.raises(ValueError):
            OneHotCounter6Bits(initial_position=COUNTER_BITS)

        with pytest.raises(ValueError):
            OneHotCounter6Bits(initial_position=-1)

    def test_rotation(self) -> None:
        counter = OneHotCounter6Bits()
        expected = [
            (1, 0, 0, 0, 0, 0),
            (0, 1, 0, 0, 0, 0),
            (0, 0, 1, 0, 0, 0),
            (0, 0, 0, 1, 0, 0),
            (0, 0, 0, 0, 1, 0),
            (0, 0, 0, 0, 0, 1),
            (1, 0, 0, 0, 0, 0),
        ]

        for state in expected:
            counter(0)
            assert counter(1) == state

    def test_transistor_count(self) -> None:
        assert OneHotCounter6Bits().transistor_count == 276
        assert OneHotCounter6Bits().bit_count == 2 * COUNTER_BITS
