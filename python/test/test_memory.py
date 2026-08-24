"""
test_memory.py
==============

Tests for the storage components in ``pytranscpu/memory.py``.
"""

from __future__ import annotations

from pytranscpu.memory import ProgramCounter4Bits, Ram256Bits, Register8Bits


class TestRegister8Bits:
    def test_write_and_read_sequence(self) -> None:
        register = Register8Bits()
        data = (1, 0, 1, 0, 1, 0, 1, 0)

        assert register((0,) * 8, 0, 0, 1) == (0,) * 8
        assert register(data, 0, 0, 1) == (0,) * 8  # clock LOW: nothing written
        assert register(data, 1, 0, 1) == (0,) * 8  # save LOW: nothing written
        assert register(data, 0, 0, 1) == (0,) * 8
        assert register(data, 1, 1, 1) == data  # rising edge with save HIGH
        assert register(data, 0, 0, 1) == data  # the value is kept
        assert register(data, 0, 1, 1) == data
        assert register(data, 0, 1, 0) == (None,) * 8  # load LOW floats

    def test_state_property(self) -> None:
        register = Register8Bits()
        assert register.state == (0,) * 8

        register((1, 1, 0, 0, 1, 0, 1, 0), 0, 1, 1)
        register((1, 1, 0, 0, 1, 0, 1, 0), 1, 1, 1)
        assert register.state == (1, 1, 0, 0, 1, 0, 1, 0)

    def test_transistor_count(self) -> None:
        assert Register8Bits().transistor_count == 544
        # Eight flip-flops of two latches: sixteen physical bits.
        assert Register8Bits().bit_count == 16


class TestProgramCounter4Bits:
    def test_count_load_and_wrap_sequence(self) -> None:
        counter = ProgramCounter4Bits()

        assert counter((0, 0, 0, 0), 0, 0, 1) == (0, 0, 0, 0)
        assert counter((0, 0, 0, 0), 1, 0, 1) == (1, 0, 0, 0)  # increment
        assert counter((0, 0, 0, 0), 0, 0, 1) == (1, 0, 0, 0)  # no edge
        assert counter((1, 1, 1, 1), 1, 1, 1) == (0, 1, 0, 0)
        assert counter((1, 1, 1, 1), 0, 1, 1) == (0, 1, 0, 0)
        assert counter((0, 0, 0, 0), 1, 1, 1) == (1, 1, 1, 1)  # load 15
        assert counter((0, 0, 0, 0), 0, 0, 1) == (1, 1, 1, 1)
        assert counter((0, 0, 0, 0), 1, 0, 1) == (0, 0, 0, 0)  # 15 + 1 wraps
        assert counter((0, 0, 0, 0), 0, 0, 1) == (0, 0, 0, 0)
        assert counter((0, 0, 0, 0), 1, 0, 1) == (1, 0, 0, 0)

    def test_load_low_floats_the_outputs(self) -> None:
        counter = ProgramCounter4Bits()
        assert counter((0, 0, 0, 1), 0, 0, 0) == (None,) * 4
        assert counter((0, 0, 0, 1), 1, 0, 0) == (None,) * 4

    def test_transistor_count(self) -> None:
        assert ProgramCounter4Bits().transistor_count == 1268
        assert ProgramCounter4Bits().bit_count == 16


class TestRam256Bits:
    def test_write_and_read_back(self) -> None:
        ram = Ram256Bits()

        # Write at address 0 with a LOW->HIGH clock sequence.
        ram((1, 0, 1, 0, 1, 0, 1, 0), (0, 0, 0, 0), 0, 1, 0)
        ram((1, 0, 1, 0, 1, 0, 1, 0), (0, 0, 0, 0), 1, 1, 0)
        assert ram((0,) * 8, (0, 0, 0, 0), 0, 0, 1) == (1, 0, 1, 0, 1, 0, 1, 0)

        # Write at address 3: LSB-first (1, 1, 0, 0).
        ram((0, 1, 0, 1, 0, 1, 0, 1), (1, 1, 0, 0), 0, 1, 0)
        ram((0, 1, 0, 1, 0, 1, 0, 1), (1, 1, 0, 0), 1, 1, 0)
        assert ram((0,) * 8, (1, 1, 0, 0), 0, 0, 1) == (0, 1, 0, 1, 0, 1, 0, 1)

        # The first cell is unchanged.
        assert ram((0,) * 8, (0, 0, 0, 0), 0, 0, 1) == (1, 0, 1, 0, 1, 0, 1, 0)

        # Never-written cells read as zero.
        assert ram((0,) * 8, (0, 0, 1, 0), 0, 0, 1) == (0,) * 8

    def test_load_low_floats_the_bus(self) -> None:
        assert Ram256Bits()((1,) * 8, (0, 0, 0, 0), 0, 0, 0) == (None,) * 8

    def test_transistor_count(self) -> None:
        assert Ram256Bits().transistor_count == 9048
        assert Ram256Bits().bit_count == 256
