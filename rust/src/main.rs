//! Benchmark binary for the SAP-1: countdown 5 -> 0 by steps of 3.
//!
//! Mirrors the Python entry point `python/bitbybit/main.py`.
//!
//! Run with:
//!
//! ```text
//! cargo run --release --manifest-path rust/Cargo.toml
//! ```

use std::time::Instant;

use sparse_energy_benchmark::cpu_sap1::Sap1;
use sparse_energy_benchmark::hardware::{bits_to_int, Bit};

const PROGRAM: [u8; 16] = [
    0x1F, // 0: LDA 0xF
    0x2E, // 1: ADD 0xE
    0x4F, // 2: STA 0xF
    0x1D, // 3: LDA 0xD
    0x3C, // 4: SUB 0xC
    0x79, // 5: JZ 9
    0x4D, // 6: STA 0xD
    0x61, // 7: JMP 1
    0x00, // 8: NOP
    0x1F, // 9: LDA 0xF
    0xF0, // A: HLT
    0x00, // B: unused
    0x01, // C: constant 1
    0x05, // D: initial count = 5
    0x03, // E: constant 3
    0x00, // F: sum initialized to 0
];

fn main() {
    let mut cpu = Sap1::default();
    println!("Number of transistors: {}", cpu.transistor_count());
    cpu.load_program(&PROGRAM).expect("program load failed");

    let start = Instant::now();
    for inst in 0..100 {
        let ir = cpu.current_instruction();
        let pc = cpu.program_counter_value();
        let acc = bits_to_int(&cpu.out(), false);
        let ram = cpu.ram_contents();
        let ram_d = bits_to_int(&ram[0xD], false);
        let ram_f = bits_to_int(&ram[0xF], false);
        let (carry, zero) = cpu.flags();
        println!(
            "Inst {:02}: PC={:X} IR={:02X} ACC={} [0xD]={} [0xF]={} Z={} C={}",
            inst,
            pc,
            ir,
            acc,
            ram_d,
            ram_f,
            zero as u8,
            carry as u8,
        );
        cpu.step_instruction().expect("execution error");
        if cpu.halted() {
            println!("Halted! Final ACC={}", bits_to_int(&cpu.out(), false));
            let elapsed = start.elapsed().as_secs_f64();
            println!("Execution time: {elapsed:.6} seconds");
            println!("Clock speed: {:.2} Hz", inst as f64 / elapsed);
            break;
        }
    }
}
