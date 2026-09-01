//! Profiling benchmark for the SAP-1 simulation.
//!
//! Run with:
//!
//! ```text
//! cargo run --release --example bench
//! ```

use std::io::{self, Write};
use std::time::Instant;

use sparse_energy_benchmark::arithmetic::{Adder8Bits, ALU8Bits};
use sparse_energy_benchmark::cpu_sap1::Sap1;
use sparse_energy_benchmark::decoder::Decoder4to16;
use sparse_energy_benchmark::hardware::{Bit, Component, Signal};
use sparse_energy_benchmark::memory::{ProgramCounter4Bits, Ram256Bits, Register8Bits};
use sparse_energy_benchmark::mux::Mux8bits2x1;

const PROGRAM: [u8; 16] = [
    0x1F, 0x2E, 0x4F, 0x1D, 0x3C, 0x79, 0x4D, 0x61, 0x00, 0x1F, 0xF0, 0x00, 0x01, 0x05, 0x03, 0x00,
];

fn d(bits: &[Bit]) -> Vec<Signal> {
    bits.iter().copied().map(Signal::from).collect()
}

fn bench<F: FnMut()>(label: &str, mut f: F) {
    // warmup + timing
    for _ in 0..200 {
        f();
    }
    let n = 1_000u64;
    let start = Instant::now();
    for _ in 0..n {
        f();
    }
    let per_call = start.elapsed().as_secs_f64() * 1e9 / n as f64;
    println!("{:<28} {:>10.1} ns/call", label, per_call);
    let _ = io::stdout().flush();
}

fn main() {
    println!("=== Micro-benchmarks (ns per conduct call) ===");

    let one = |b: bool| if b { Bit::High } else { Bit::Low };

    let decoder = Decoder4to16::default();
    bench("Decoder4to16", || {
        let _ = decoder.conduct(&d(&[one(true), one(false), one(true), one(false)]));
    });

    let reg = Register8Bits::default();
    bench("Register8Bits (save=High)", || {
        let mut inputs = d(&[one(true), one(false), one(true), one(false), one(true), one(false), one(true), one(false)]);
        inputs.extend([Signal::from(Bit::Low), Signal::from(Bit::High), Signal::from(Bit::High)]);
        let _ = reg.conduct(&inputs);
    });

    let adder = Adder8Bits::default();
    bench("Adder8Bits", || {
        let mut inputs = d(&[one(true), one(false), one(true), one(false), one(true), one(false), one(true), one(false)]);
        inputs.extend(d(&[one(false), one(true), one(false), one(true), one(false), one(true), one(false), one(true)]));
        inputs.push(Signal::from(Bit::Low));
        let _ = adder.conduct(&inputs);
    });

    let mux = Mux8bits2x1::default();
    bench("Mux8bits2x1", || {
        let mut inputs = d(&[one(true), one(false), one(true), one(false), one(true), one(false), one(true), one(false)]);
        inputs.extend(d(&[one(false), one(true), one(false), one(true), one(false), one(true), one(false), one(true)]));
        inputs.push(Signal::from(Bit::High));
        let _ = mux.conduct(&inputs);
    });

    let alu = ALU8Bits::default();
    bench("ALU8Bits (eo=High)", || {
        let mut inputs = d(&[one(true), one(false), one(true), one(false), one(true), one(false), one(true), one(false)]);
        inputs.extend(d(&[one(false), one(true), one(false), one(true), one(false), one(true), one(false), one(true)]));
        inputs.extend([Signal::from(Bit::Low), Signal::from(Bit::High)]);
        let _ = alu.conduct(&inputs);
    });

    let pc = ProgramCounter4Bits::default();
    bench("ProgramCounter4Bits", || {
        let mut inputs = d(&[one(true), one(false), one(true), one(false)]);
        inputs.extend([Signal::from(Bit::Low), Signal::from(Bit::Low), Signal::from(Bit::High)]);
        let _ = pc.conduct(&inputs);
    });

    let ram = Ram256Bits::default();
    bench("Ram256Bits (read)", || {
        let mut inputs = d(&[one(true), one(false), one(true), one(false), one(true), one(false), one(true), one(false)]);
        inputs.extend(d(&[one(true), one(false), one(true), one(false)]));
        inputs.extend([Signal::from(Bit::Low), Signal::from(Bit::Low), Signal::from(Bit::High)]);
        let _ = ram.conduct(&inputs);
    });

    println!("\n=== Whole-program throughput ===");
    let mut cpu = Sap1::default();
    cpu.load_program(&PROGRAM).expect("program load failed");

    // One full run first to count ticks.
    let mut ticks = 0u64;
    loop {
        cpu.clock_tick().expect("execution error");
        ticks += 1;
        if cpu.halted() {
            break;
        }
    }
    println!("ticks per full program run: {ticks}");

    let runs = 5u64;
    let start = Instant::now();
    for _ in 0..runs {
        let mut cpu = Sap1::default();
        cpu.load_program(&PROGRAM).expect("program load failed");
        loop {
            cpu.clock_tick().expect("execution error");
            if cpu.halted() {
                break;
            }
        }
    }
    let total = start.elapsed().as_secs_f64();
    let total_ticks = runs * ticks;
    println!(
        "{runs} runs, {total_ticks} ticks in {total:.3} s -> {:.0} ns/tick ({:.0} kHz)",
        total * 1e9 / total_ticks as f64,
        total_ticks as f64 / total / 1e3,
    );
    let _ = io::stdout().flush();
}
