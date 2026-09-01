//! SAP-1 CPU simulated at the transistor level — Rust implementation.
//!
//! Fully independent from the Python implementation living in `../python`.

pub mod alu_sap2;
pub mod arithmetic;
pub mod cpu_sap1;
pub mod decoder;
pub mod gates;
pub mod hardware;
pub mod latches;
pub mod memory;
pub mod mux;
pub mod utils;

/// Compile-time switch enabling the lossless fast paths (skipping
/// provably-inactive transistors). Mirrors `bitbybit.FAST` on the
/// Python side.
pub const FAST: bool = true;
