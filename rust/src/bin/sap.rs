//! CLI: assemble a text program file and run it on SAP-1 or SAP-2.
//!
//! ```text
//! cargo install --path rust
//! sap sap2 programs/sap2_demo.txt
//! sap sap1 programs/sap1_demo.txt
//! ```

use std::process::ExitCode;
use std::time::Instant;

use bitbybit::asm::{assemble_sap1, assemble_sap2};
use bitbybit::cpu_sap1::Sap1;
use bitbybit::cpu_sap2::Sap2;
use bitbybit::utils::bits_to_int;

const MAX_STEPS: usize = 10_000;

const HELP: &str = "Assemble a text program and run it on a transistor-level CPU.

usage: sap <sap1|sap2> <file>

The file holds one instruction per line, `;` comments, blank lines ok,
numbers decimal or 0x hex, DB emits raw data bytes:

    MVIB 3     ; sap2: B = 3
    LDA 0x10   ; sap1: A = MEM[14]
    JMP 4      ; hand-counted byte offset (no labels yet)

Examples:
    sap sap2 programs/sap2_demo.txt
    sap sap1 programs/sap1_demo.txt";

fn fail(message: String) -> ExitCode {
    eprintln!("error: {message}");
    ExitCode::FAILURE
}

/// Timing footer shared by both CPUs.
fn print_stats(instructions: u64, ticks: u64, elapsed: f64) {
    println!("instructions: {instructions}");
    println!("ticks: {ticks}");
    println!("time: {elapsed:.6} seconds");
    println!("frequency: {:.0} Hz", instructions as f64 / elapsed);
}

fn help() -> ExitCode {
    println!("{HELP}");
    ExitCode::SUCCESS
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() == 2 && (args[1] == "-h" || args[1] == "--help") {
        return help();
    }
    if args.len() != 3 || (args[1] != "sap1" && args[1] != "sap2") {
        eprintln!("usage: sap <sap1|sap2> <file>");
        return ExitCode::FAILURE;
    }
    let source = match std::fs::read_to_string(&args[2]) {
        Ok(source) => source,
        Err(err) => return fail(format!("cannot read {}: {err}", args[2])),
    };

    if args[1] == "sap1" {
        let program = match assemble_sap1(&source) {
            Ok(program) => program,
            Err(err) => return fail(err),
        };
        let mut cpu = Sap1::default();
        let start = Instant::now();
        if let Err(err) = cpu.load_program(&program).and_then(|()| cpu.run(MAX_STEPS)) {
            return fail(format!("execution failed: {err:?}"));
        }
        let elapsed = start.elapsed().as_secs_f64();
        println!("halted: {}", cpu.halted());
        println!("acc: {}", bits_to_int(&cpu.out(), false));
        print_stats(cpu.instruction_count(), cpu.tick_count(), elapsed);
    } else {
        let program = match assemble_sap2(&source) {
            Ok(program) => program,
            Err(err) => return fail(err),
        };
        let mut cpu = Sap2::default();
        let start = Instant::now();
        if let Err(err) = cpu.load_program(&program).and_then(|()| cpu.run(MAX_STEPS)) {
            return fail(format!("execution failed: {err:?}"));
        }
        let elapsed = start.elapsed().as_secs_f64();
        let (zero, carry, sign) = cpu.flags();
        println!("halted: {}", cpu.halted());
        println!("acc: {}", bits_to_int(&cpu.out(), false));
        println!("out: {}", bits_to_int(&cpu.output(), false));
        println!("flags: Z={zero:?} C={carry:?} S={sign:?}");
        print_stats(cpu.instruction_count(), cpu.tick_count(), elapsed);
    }
    ExitCode::SUCCESS
}
