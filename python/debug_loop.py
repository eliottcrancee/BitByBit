import time

from pytranscpu.cpu_sap1 import SAP1
from pytranscpu.hardware import bits_to_int

cpu = SAP1()
print("Number of transistors:", cpu.transistor_count)
program = [
    0x1F,  # 0: LDA 0xF
    0x2E,  # 1: ADD 0xE
    0x4F,  # 2: STA 0xF
    0x1D,  # 3: LDA 0xD
    0x3C,  # 4: SUB 0xC
    0x79,  # 5: JZ 9
    0x4D,  # 6: STA 0xD
    0x61,  # 7: JMP 1
    0x00,  # 8: NOP
    0x1F,  # 9: LDA 0xF
    0xF0,  # A: HLT
    0x00,  # B: unused
    0x01,  # C: constant 1
    0x05,  # D: initial count = 5
    0x03,  # E: constant 3
    0x00,  # F: sum initialized to 0
]
cpu.load_program(program)

start = time.perf_counter()
for inst in range(100):
    ir = bits_to_int(cpu.instruction_register.state)
    pc = bits_to_int(cpu.program_counter.state[:4])
    acc = bits_to_int(cpu.accumulator.state)
    ram_d = bits_to_int(cpu.ram.registers[0xD].state)
    ram_f = bits_to_int(cpu.ram.registers[0xF].state)
    flags = f"Z={cpu.zero_flag.state[0]} C={cpu.carry_flag.state[0]}"
    print(
        f"Inst {inst:02d}: PC={pc:X} IR={ir:02X} ACC={acc} [0xD]={ram_d} [0xF]={ram_f} {flags}"
    )
    cpu.step_instruction()
    if cpu.halted:
        print(f"Halted! Final ACC={bits_to_int(cpu.accumulator.state)}")
        stop = time.perf_counter()
        print(f"Execution time: {stop - start:.6f} seconds")
        print(f"Clock speed: {inst / (stop - start):.2f} Hz")
        break
