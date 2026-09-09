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

use bitbybit::asm::assemble_sap1;
use bitbybit::cpu_sap1::Sap1;
use bitbybit::utils::bits_to_int;

/// Same nested countdown loops (15 x 15) as text: the assembler is
/// dogfooded here, so `main` also tests it end to end.
const PROGRAM_TEXT: &str = "
    LDA 0xE ; 0: load outer count
    SUB 0xC ; 1: outer -= 1
    STA 0xE ; 2
    JZ 0xB  ; 3: outer reached 0 -> HLT (SUB sets Z, not LDA)
    LDI 0xF ; 4: reload inner count = 15
    STA 0xD ; 5
    LDA 0xD ; 6: inner loop
    SUB 0xC ; 7: inner -= 1
    STA 0xD ; 8
    JZ 0    ; 9: inner done -> outer loop
    JMP 6   ; A
    HLT     ; B
    DB 0x01 ; C: constant 1
    DB 0x0F ; D: initial inner count = 15
    DB 0x0F ; E: outer count = 15
    DB 0x00 ; F: unused
";

/// Runs the program once and returns the number of instructions executed.
fn run_once() -> u64 {
    let program: [u8; 16] = assemble_sap1(PROGRAM_TEXT)
        .expect("assembly failed")
        .try_into()
        .expect("program must be 16 bytes");
    let mut cpu = Sap1::default();
    cpu.load_program(&program).expect("program load failed");

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
    const RUNS: u32 = 30;

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
