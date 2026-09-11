//! Tiny assemblers: readable mnemonic text -> program bytes.
//!
//! Same shape for both CPUs so they stay aligned: each takes source
//! text (one instruction per line, `;` comments, blank lines ok,
//! numbers decimal or `0x` hex) and returns the bytes or a
//! line-numbered error. No labels, no macros — hand-counted
//! addresses, like the hardware deserves.

/// Parse one number: decimal (`15`) or hex (`0x0F`).
fn parse_num(token: &str, line: usize) -> Result<u16, String> {
    let (digits, radix) = match token.strip_prefix("0x").or(token.strip_prefix("0X")) {
        Some(hex) => (hex, 16),
        None => (token, 10),
    };
    u16::from_str_radix(digits, radix).map_err(|_| format!("line {line}: bad number `{token}`"))
}

/// Split source into `(line_number, mnemonic, operands)` rows.
fn rows(source: &str) -> Vec<(usize, String, Vec<String>)> {
    source
        .lines()
        .enumerate()
        .filter_map(|(index, raw)| {
            let code = raw.split(';').next().unwrap_or("").trim();
            if code.is_empty() {
                return None;
            }
            let mut parts = code.split_whitespace();
            let mnemonic = parts.next().unwrap_or("").to_ascii_uppercase();
            Some((index + 1, mnemonic, parts.map(str::to_string).collect()))
        })
        .collect()
}

/// Assemble a SAP-1 program: `LDA 0xE`, `LDI 5`, `JZ 0`, `NOP`,
/// `HLT`, `DB 0x01` (raw bytes for data).
pub fn assemble_sap1(source: &str) -> Result<Vec<u8>, String> {
    fn op_byte(op: u8, operand: &[String], line: usize) -> Result<u8, String> {
        if operand.len() != 1 {
            return Err(format!("line {line}: one address expected"));
        }
        let addr = parse_num(&operand[0], line)?;
        if addr > 0xF {
            return Err(format!("line {line}: address out of range `{addr}`"));
        }
        Ok(op << 4 | addr as u8)
    }

    let mut bytes = Vec::new();
    for (line, mnemonic, operand) in rows(source) {
        match mnemonic.as_str() {
            "NOP" => bytes.push(0x00),
            "HLT" => bytes.push(0xF0),
            "LDA" => bytes.push(op_byte(0x1, &operand, line)?),
            "ADD" => bytes.push(op_byte(0x2, &operand, line)?),
            "SUB" => bytes.push(op_byte(0x3, &operand, line)?),
            "STA" => bytes.push(op_byte(0x4, &operand, line)?),
            "LDI" => bytes.push(op_byte(0x5, &operand, line)?),
            "JMP" => bytes.push(op_byte(0x6, &operand, line)?),
            "JZ" => bytes.push(op_byte(0x7, &operand, line)?),
            "JC" => bytes.push(op_byte(0x8, &operand, line)?),
            "DB" => {
                if operand.is_empty() {
                    return Err(format!("line {line}: DB needs at least one byte"));
                }
                for token in &operand {
                    let value = parse_num(token, line)?;
                    if value > 0xFF {
                        return Err(format!("line {line}: byte out of range `{value}`"));
                    }
                    bytes.push(value as u8);
                }
            }
            _ => return Err(format!("line {line}: unknown instruction `{mnemonic}`")),
        }
    }
    Ok(bytes)
}

/// Assemble a SAP-2 program: `MVIA 10`, `ADD B`, `LDA 0x0010`,
/// `JMP 6`, `IN`, `OUT`, `RET`, `HLT`, `DB 0x01` (raw data bytes).
pub fn assemble_sap2(source: &str) -> Result<Vec<u8>, String> {
    fn reg_op(mnemonic: &str, operand: &[String], line: usize, byte: u8) -> Result<u8, String> {
        match operand {
            [] => Ok(byte),
            [reg] if reg.to_ascii_uppercase() == "B" => Ok(byte),
            _ => Err(format!("line {line}: `{mnemonic}` takes `B` or nothing")),
        }
    }

    fn with_u8(mnemonic: &str, operand: &[String], line: usize, op: u8) -> Result<Vec<u8>, String> {
        if operand.len() != 1 {
            return Err(format!("line {line}: `{mnemonic}` takes one byte"));
        }
        let value = parse_num(&operand[0], line)?;
        if value > 0xFF {
            return Err(format!("line {line}: byte out of range `{value}`"));
        }
        Ok(vec![op, value as u8])
    }

    fn with_addr(
        mnemonic: &str,
        operand: &[String],
        line: usize,
        op: u8,
    ) -> Result<Vec<u8>, String> {
        if operand.len() != 1 {
            return Err(format!("line {line}: `{mnemonic}` takes one address"));
        }
        let addr = parse_num(&operand[0], line)?;
        Ok(vec![op, (addr & 0xFF) as u8, (addr >> 8) as u8])
    }

    let mut bytes = Vec::new();
    for (line, mnemonic, operand) in rows(source) {
        match mnemonic.as_str() {
            "NOP" => bytes.push(0x00),
            "HLT" => bytes.push(0xF0),
            "ADD" => bytes.push(reg_op(&mnemonic, &operand, line, 0x50)?),
            "SUB" => bytes.push(reg_op(&mnemonic, &operand, line, 0x51)?),
            "ANA" => bytes.push(reg_op(&mnemonic, &operand, line, 0x52)?),
            "ORA" => bytes.push(reg_op(&mnemonic, &operand, line, 0x53)?),
            "XRA" => bytes.push(reg_op(&mnemonic, &operand, line, 0x54)?),
            "CMA" => bytes.push(reg_op(&mnemonic, &operand, line, 0x55)?),
            "INRA" => bytes.push(reg_op(&mnemonic, &operand, line, 0x56)?),
            "DCRA" => bytes.push(reg_op(&mnemonic, &operand, line, 0x57)?),
            "IN" => bytes.push(reg_op(&mnemonic, &operand, line, 0x60)?),
            "OUT" => bytes.push(reg_op(&mnemonic, &operand, line, 0x61)?),
            "RET" => bytes.push(reg_op(&mnemonic, &operand, line, 0xC0)?),
            "PSHA" => bytes.push(reg_op(&mnemonic, &operand, line, 0xC1)?),
            "POPA" => bytes.push(reg_op(&mnemonic, &operand, line, 0xC2)?),
            "MVIA" => bytes.extend(with_u8(&mnemonic, &operand, line, 0x30)?),
            "MVIB" => bytes.extend(with_u8(&mnemonic, &operand, line, 0x40)?),
            "LDA" => bytes.extend(with_addr(&mnemonic, &operand, line, 0x10)?),
            "STA" => bytes.extend(with_addr(&mnemonic, &operand, line, 0x20)?),
            "JMP" => bytes.extend(with_addr(&mnemonic, &operand, line, 0x70)?),
            "JZ" => bytes.extend(with_addr(&mnemonic, &operand, line, 0x80)?),
            "JNZ" => bytes.extend(with_addr(&mnemonic, &operand, line, 0x90)?),
            "JM" => bytes.extend(with_addr(&mnemonic, &operand, line, 0xA0)?),
            "CALL" => bytes.extend(with_addr(&mnemonic, &operand, line, 0xB0)?),
            "DB" => {
                if operand.is_empty() {
                    return Err(format!("line {line}: DB needs at least one byte"));
                }
                for token in &operand {
                    let value = parse_num(token, line)?;
                    if value > 0xFF {
                        return Err(format!("line {line}: byte out of range `{value}`"));
                    }
                    bytes.push(value as u8);
                }
            }
            _ => return Err(format!("line {line}: unknown instruction `{mnemonic}`")),
        }
    }
    Ok(bytes)
}

#[cfg(test)]
mod sap1_tests {
    use super::*;

    #[test]
    fn test_assembles_every_mnemonic() {
        assert_eq!(
            assemble_sap1("NOP\nLDA 0xE\nADD 14\nSUB 3\nSTA 4\nLDI 5\nJMP 6\nJZ 7\nJC 8\nHLT"),
            Ok(vec![
                0x00, 0x1E, 0x2E, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0xF0
            ])
        );
    }

    #[test]
    fn test_comments_blanks_case_and_db() {
        assert_eq!(
            assemble_sap1("; full line\n\n  ldi 10 ; trailing\nDB 0x01 2"),
            Ok(vec![0x5A, 0x01, 0x02])
        );
    }

    #[test]
    fn test_rejects_bad_lines() {
        assert!(assemble_sap1("LDA").is_err());
        assert!(assemble_sap1("LDA 16").is_err());
        assert!(assemble_sap1("FOO").is_err());
    }
}

#[cfg(test)]
mod sap2_tests {
    use super::*;
    use crate::cpu_sap2::Sap2;
    use crate::utils::bits_to_int;

    #[test]
    fn test_assembles_every_mnemonic() {
        assert_eq!(
            assemble_sap2(
                "NOP\nLDA 0x0010\nSTA 16\nMVIA 1\nMVIB 2\nADD B\nSUB\nANA B\nORA B\n\
                 XRA B\nCMA\nINRA\nDCRA\nIN\nOUT\nJMP 6\nJZ 7\nJNZ 8\nJM 9\n\
                 CALL 0x0100\nRET\nPSHA\nPOPA\nHLT\nDB 0xFF"
            ),
            Ok(vec![
                0x00, 0x10, 0x10, 0x00, 0x20, 0x10, 0x00, 0x30, 0x01, 0x40, 0x02, 0x50, 0x51, 0x52,
                0x53, 0x54, 0x55, 0x56, 0x57, 0x60, 0x61, 0x70, 0x06, 0x00, 0x80, 0x07, 0x00, 0x90,
                0x08, 0x00, 0xA0, 0x09, 0x00, 0xB0, 0x00, 0x01, 0xC0, 0xC1, 0xC2, 0xF0, 0xFF,
            ])
        );
    }

    #[test]
    fn test_rejects_bad_lines() {
        assert!(assemble_sap2("MVIA").is_err());
        assert!(assemble_sap2("LDA 0x10000").is_err());
        assert!(assemble_sap2("ADD C").is_err());
        assert!(assemble_sap2("FOO").is_err());
    }

    #[test]
    fn test_assembled_program_runs() {
        let program = assemble_sap2("MVIB 3\nMVIA 5\nADD B\nHLT").expect("assembly failed");
        let mut cpu = Sap2::default();
        cpu.load_program(&program).expect("load failed");
        cpu.run(100).expect("run failed");
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 8);
    }

    #[test]
    fn test_stack_program_runs() {
        // programs/sap2_stack.txt: push/pop plus a CALL/RET subroutine.
        let source = include_str!("../../programs/sap2_stack.txt");
        let program = assemble_sap2(source).expect("assembly failed");
        let mut cpu = Sap2::default();
        cpu.load_program(&program).expect("load failed");
        cpu.run(200).expect("run failed");
        assert!(cpu.halted());
        assert_eq!(bits_to_int(&cpu.out(), false), 4);
    }
}
