//! Logic gates built directly from [`crate::hardware::Pmos`] and
//! [`crate::hardware::Nmos`] transistors in a complementary
//! pull-up / pull-down arrangement (CMOS).
//!
//! Each gate is the smallest possible composition of transistors:
//!
//! * NOT  — 1 PMOS + 1 NMOS (2T)
//! * NAND / NOR — 2 PMOS + 2 NMOS (4T)
//! * AND = NAND + NOT and OR = NOR + NOT (6T)
//! * XOR — four NAND gates (16T), XNOR = XOR + NOT (18T)

use crate::hardware::{Bit, Component, HardwareError, Nmos, Pmos, Signal, wire};

/// Inverter: 1 PMOS pulling up, 1 NMOS pulling down.
#[derive(Debug, Default)]
pub struct NotGate {
    pmos: Pmos,
    nmos: Nmos,
}

impl Component for NotGate {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [input] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 1,
                actual: inputs.len(),
            });
        };
        let pmos_output = self.pmos.conduct(&[*input, Signal::Driven(Bit::High)])?;
        let nmos_output = self.nmos.conduct(&[*input, Signal::Driven(Bit::Low)])?;
        Ok(vec![wire(pmos_output.into_iter().chain(nmos_output))?])
    }

    fn transistor_count(&self) -> usize {
        self.pmos.transistor_count() + self.nmos.transistor_count()
    }
}

/// NAND: parallel PMOS pull-up, series NMOS pull-down (4T).
#[derive(Debug, Default)]
pub struct NandGate {
    pmos_a: Pmos,
    pmos_b: Pmos,
    nmos_a: Nmos,
    nmos_b: Nmos,
}

impl Component for NandGate {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };
        let pmos_a_output = self.pmos_a.conduct(&[*a, Signal::Driven(Bit::High)])?;
        let pmos_b_output = self.pmos_b.conduct(&[*b, Signal::Driven(Bit::High)])?;
        let pull_up = wire(pmos_a_output.into_iter().chain(pmos_b_output))?;
        let nmos_a_output = self.nmos_a.conduct(&[*a, Signal::Driven(Bit::Low)])?;
        let nmos_b_output = self.nmos_b.conduct(&[*b, nmos_a_output[0]])?;
        Ok(vec![wire([pull_up, nmos_b_output[0]])?])
    }

    fn transistor_count(&self) -> usize {
        self.pmos_a.transistor_count()
            + self.pmos_b.transistor_count()
            + self.nmos_a.transistor_count()
            + self.nmos_b.transistor_count()
    }
}

/// NOR: series PMOS pull-up, parallel NMOS pull-down (4T).
#[derive(Debug, Default)]
pub struct NorGate {
    pmos_a: Pmos,
    pmos_b: Pmos,
    nmos_a: Nmos,
    nmos_b: Nmos,
}

impl Component for NorGate {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };
        let pmos_a_output = self.pmos_a.conduct(&[*a, Signal::Driven(Bit::High)])?;
        let pmos_b_output = self.pmos_b.conduct(&[*b, pmos_a_output[0]])?;
        let nmos_a_output = self.nmos_a.conduct(&[*a, Signal::Driven(Bit::Low)])?;
        let nmos_b_output = self.nmos_b.conduct(&[*b, Signal::Driven(Bit::Low)])?;
        let pull_down = wire(nmos_a_output.into_iter().chain(nmos_b_output))?;
        Ok(vec![wire([pmos_b_output[0], pull_down])?])
    }

    fn transistor_count(&self) -> usize {
        self.pmos_a.transistor_count()
            + self.pmos_b.transistor_count()
            + self.nmos_a.transistor_count()
            + self.nmos_b.transistor_count()
    }
}

/// AND = NAND followed by an inverter (6T).
#[derive(Debug, Default)]
pub struct AndGate {
    nand: NandGate,
    not: NotGate,
}

impl Component for AndGate {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };
        let nand_output = self.nand.conduct(&[*a, *b])?;
        self.not.conduct(&nand_output)
    }

    fn transistor_count(&self) -> usize {
        self.nand.transistor_count() + self.not.transistor_count()
    }
}

/// OR = NOR followed by an inverter (6T).
#[derive(Debug, Default)]
pub struct OrGate {
    nor: NorGate,
    not: NotGate,
}

impl Component for OrGate {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };
        let nor_output = self.nor.conduct(&[*a, *b])?;
        self.not.conduct(&nor_output)
    }

    fn transistor_count(&self) -> usize {
        self.nor.transistor_count() + self.not.transistor_count()
    }
}

#[derive(Debug, Default)]
pub struct XorGate {
    nand1: NandGate,
    nand2: NandGate,
    nand3: NandGate,
    nand4: NandGate,
}

impl Component for XorGate {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };
        let nand1_output = self.nand1.conduct(&[*a, *b])?;
        let nand2_output = self.nand2.conduct(&[*a, nand1_output[0]])?;
        let nand3_output = self.nand3.conduct(&[*b, nand1_output[0]])?;
        self.nand4.conduct(&[nand2_output[0], nand3_output[0]])
    }

    fn transistor_count(&self) -> usize {
        self.nand1.transistor_count()
            + self.nand2.transistor_count()
            + self.nand3.transistor_count()
            + self.nand4.transistor_count()
    }
}

/// XNOR = XOR followed by an inverter (18T).
#[derive(Debug, Default)]
pub struct XnorGate {
    xor: XorGate,
    not: NotGate,
}

impl Component for XnorGate {
    fn conduct(&self, inputs: &[Signal]) -> Result<Vec<Signal>, HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };
        let xor_output = self.xor.conduct(&[*a, *b])?;
        self.not.conduct(&xor_output)
    }

    fn transistor_count(&self) -> usize {
        self.xor.transistor_count() + self.not.transistor_count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_not() {
        let gate = NotGate::default();
        assert_eq!(gate.compute(&[Bit::Low]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::High]), Ok(vec![Bit::Low]));
        assert_eq!(gate.transistor_count(), 2);
    }

    #[test]
    fn test_nand() {
        let gate = NandGate::default();
        assert_eq!(gate.compute(&[Bit::Low, Bit::Low]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::Low, Bit::High]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::High, Bit::Low]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::High, Bit::High]), Ok(vec![Bit::Low]));
        assert_eq!(gate.transistor_count(), 4);
    }

    #[test]
    fn test_nor() {
        let gate = NorGate::default();
        assert_eq!(gate.compute(&[Bit::Low, Bit::Low]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::Low, Bit::High]), Ok(vec![Bit::Low]));
        assert_eq!(gate.compute(&[Bit::High, Bit::Low]), Ok(vec![Bit::Low]));
        assert_eq!(gate.compute(&[Bit::High, Bit::High]), Ok(vec![Bit::Low]));
        assert_eq!(gate.transistor_count(), 4);
    }

    #[test]
    fn test_and() {
        let gate = AndGate::default();
        assert_eq!(gate.compute(&[Bit::Low, Bit::Low]), Ok(vec![Bit::Low]));
        assert_eq!(gate.compute(&[Bit::Low, Bit::High]), Ok(vec![Bit::Low]));
        assert_eq!(gate.compute(&[Bit::High, Bit::Low]), Ok(vec![Bit::Low]));
        assert_eq!(gate.compute(&[Bit::High, Bit::High]), Ok(vec![Bit::High]));
        assert_eq!(gate.transistor_count(), 6);
    }

    #[test]
    fn test_or() {
        let gate = OrGate::default();
        assert_eq!(gate.compute(&[Bit::Low, Bit::Low]), Ok(vec![Bit::Low]));
        assert_eq!(gate.compute(&[Bit::Low, Bit::High]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::High, Bit::Low]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::High, Bit::High]), Ok(vec![Bit::High]));
        assert_eq!(gate.transistor_count(), 6);
    }

    #[test]
    fn test_xor() {
        let gate = XorGate::default();
        assert_eq!(gate.compute(&[Bit::Low, Bit::Low]), Ok(vec![Bit::Low]));
        assert_eq!(gate.compute(&[Bit::Low, Bit::High]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::High, Bit::Low]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::High, Bit::High]), Ok(vec![Bit::Low]));
        assert_eq!(gate.transistor_count(), 16);
    }

    #[test]
    fn test_xnor() {
        let gate = XnorGate::default();
        assert_eq!(gate.compute(&[Bit::Low, Bit::Low]), Ok(vec![Bit::High]));
        assert_eq!(gate.compute(&[Bit::Low, Bit::High]), Ok(vec![Bit::Low]));
        assert_eq!(gate.compute(&[Bit::High, Bit::Low]), Ok(vec![Bit::Low]));
        assert_eq!(gate.compute(&[Bit::High, Bit::High]), Ok(vec![Bit::High]));
        assert_eq!(gate.transistor_count(), 18);
    }
}
