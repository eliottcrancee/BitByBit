//! Benchmark binary for the SAP-1: nested countdown loops (15 x 15).
//!
//! Runs the program many times back-to-back without any I/O inside the
//! timed loop, so the measurement reflects pure CPU simulation speed.
//!
//! Run with:
//!
//! ```text
//! cargo run --release --manifest-path rust/Cargo.toml
//! ```

use std::time::Instant;

use sparse_energy_benchmark::cpu_sap1::Sap1;
use sparse_energy_benchmark::utils::bits_to_int;

const PROGRAM: [u8; 16] = [
    0x1E, // 0: LDA 0xE  (load outer count)
    0x7B, // 1: JZ 0xB   (outer done -> HLT)
    0x3C, // 2: SUB 0xC  (outer -= 1)
    0x4E, // 3: STA 0xE
    0x5F, // 4: LDI 0xF  (reload inner count = 15)
    0x4D, // 5: STA 0xD
    0x1D, // 6: LDA 0xD  (inner loop)
    0x3C, // 7: SUB 0xC  (inner -= 1)
    0x4D, // 8: STA 0xD
    0x70, // 9: JZ 0     (inner done -> outer loop)
    0x66, // A: JMP 6
    0xF0, // B: HLT
    0x00, // C: unused
    0x01, // D: constant 1
    0x0F, // E: outer count = 15
    0x0F, // F: inner count = 15
];

/// Runs the program once and returns the number of instructions executed.
fn run_once() -> u64 {
    let mut cpu = Sap1::default();
    cpu.load_program(&PROGRAM).expect("program load failed");

    let mut instructions = 0u64;
    while !cpu.halted() {
        cpu.step_instruction().expect("execution error");
        instructions += 1;
    }

    // Sanity check: the accumulator must be 0 when the machine halts.
    assert_eq!(bits_to_int(&cpu.out(), false), 0, "unexpected final ACC");
    instructions
}

fn main() {
    const RUNS: u32 = 1;

    let start = Instant::now();
    let mut total_instructions = 0u64;
    for _ in 0..RUNS {
        total_instructions += run_once();
    }
    let elapsed = start.elapsed().as_secs_f64();

    println!("Runs: {RUNS}");
    println!("Total instructions: {total_instructions}");
    println!("Execution time: {elapsed:.6} seconds");
    println!("Clock speed: {:.0} Hz", total_instructions as f64 / elapsed);
}
