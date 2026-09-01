//! Byte↔bit conversion helpers.
//!
//! Pure arithmetic conveniences around [`crate::hardware::Bit`]:
//! unlike everything in [`crate::hardware`], they model no electrical
//! behavior (no transistor, no signal, no wire). All multi-bit values
//! in the project are least-significant bit first, which is the
//! convention encoded here.

use crate::hardware::Bit;

/// Pack up to eight bits into a byte.
///
/// When `msb_first` is false (the project-wide convention), the first
/// bit is the least significant one.
///
/// # Panics
///
/// Panics when `bits` is longer than eight.
pub fn bits_to_int(bits: &[Bit], msb_first: bool) -> u8 {
    assert!(
        bits.len() <= 8,
        "The project only manipulates 8-bit values."
    );

    let mut result: u8 = 0;

    if msb_first {
        for bit in bits {
            result = (result << 1) | *bit as u8;
        }
    } else {
        for (index, bit) in bits.iter().enumerate() {
            result |= (*bit as u8) << index;
        }
    }

    result
}

/// Expand a byte into eight bits, least significant bit first.
pub fn int_to_bits(value: u8) -> [Bit; 8] {
    core::array::from_fn(|index| {
        if (value >> index) & 1 == 1 {
            Bit::High
        } else {
            Bit::Low
        }
    })
}

#[cfg(test)]
mod bits_to_int_tests {
    use super::*;
    use crate::hardware::Bit::{High, Low};

    #[test]
    fn test_empty_is_zero() {
        assert_eq!(bits_to_int(&[], false), 0);
    }

    #[test]
    fn test_lsb_first_by_default() {
        assert_eq!(bits_to_int(&[High, Low, High], false), 5);
        assert_eq!(bits_to_int(&[High; 8], false), 255);
    }

    #[test]
    fn test_msb_first() {
        assert_eq!(bits_to_int(&[High, Low, High], true), 0b101);
        assert_eq!(
            bits_to_int(&[Low, Low, Low, Low, High, Low, High, Low], true),
            0x0A
        );
    }
}

#[cfg(test)]
mod int_to_bits_tests {
    use super::*;

    #[test]
    fn test_expands_lsb_first() {
        assert_eq!(
            int_to_bits(0b0000_1010),
            [
                Bit::Low,  // 2^0
                Bit::High, // 2^1
                Bit::Low,  // 2^2
                Bit::High, // 2^3
                Bit::Low,  // 2^4
                Bit::Low,  // 2^5
                Bit::Low,  // 2^6
                Bit::Low,  // 2^7
            ]
        );
    }

    #[test]
    fn test_round_trip_with_bits_to_int() {
        for value in 0..=u8::MAX {
            assert_eq!(bits_to_int(&int_to_bits(value), false), value);
        }
    }
}