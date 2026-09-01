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
    const INPUTS: usize = 1;
    const OUTPUTS: usize = 1;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [input] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 1,
                actual: inputs.len(),
            });
        };

        let mut pmos_output = [Signal::HighImpedance];
        self.pmos
            .conduct_into(&[*input, Signal::Driven(Bit::High)], &mut pmos_output)?;
        let mut nmos_output = [Signal::HighImpedance];
        self.nmos
            .conduct_into(&[*input, Signal::Driven(Bit::Low)], &mut nmos_output)?;

        outputs[0] = wire(pmos_output.into_iter().chain(nmos_output))?;
        Ok(())
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
    const INPUTS: usize = 2;
    const OUTPUTS: usize = 1;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let mut pmos_a_output = [Signal::HighImpedance];
        self.pmos_a
            .conduct_into(&[*a, Signal::Driven(Bit::High)], &mut pmos_a_output)?;
        let mut pmos_b_output = [Signal::HighImpedance];
        self.pmos_b
            .conduct_into(&[*b, Signal::Driven(Bit::High)], &mut pmos_b_output)?;
        let pull_up = wire([pmos_a_output[0], pmos_b_output[0]])?;

        let mut nmos_a_output = [Signal::HighImpedance];
        self.nmos_a
            .conduct_into(&[*a, Signal::Driven(Bit::Low)], &mut nmos_a_output)?;
        let mut nmos_b_output = [Signal::HighImpedance];
        self.nmos_b
            .conduct_into(&[*b, nmos_a_output[0]], &mut nmos_b_output)?;

        outputs[0] = wire([pull_up, nmos_b_output[0]])?;
        Ok(())
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
    const INPUTS: usize = 2;
    const OUTPUTS: usize = 1;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let mut pmos_a_output = [Signal::HighImpedance];
        self.pmos_a
            .conduct_into(&[*a, Signal::Driven(Bit::High)], &mut pmos_a_output)?;
        let mut pmos_b_output = [Signal::HighImpedance];
        self.pmos_b
            .conduct_into(&[*b, pmos_a_output[0]], &mut pmos_b_output)?;

        let mut nmos_a_output = [Signal::HighImpedance];
        self.nmos_a
            .conduct_into(&[*a, Signal::Driven(Bit::Low)], &mut nmos_a_output)?;
        let mut nmos_b_output = [Signal::HighImpedance];
        self.nmos_b
            .conduct_into(&[*b, Signal::Driven(Bit::Low)], &mut nmos_b_output)?;
        let pull_down = wire([nmos_a_output[0], nmos_b_output[0]])?;

        outputs[0] = wire([pmos_b_output[0], pull_down])?;
        Ok(())
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
    const INPUTS: usize = 2;
    const OUTPUTS: usize = 1;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let mut nand_output = [Signal::HighImpedance];
        self.nand.conduct_into(&[*a, *b], &mut nand_output)?;
        self.not.conduct_into(&nand_output, outputs)
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
    const INPUTS: usize = 2;
    const OUTPUTS: usize = 1;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let mut nor_output = [Signal::HighImpedance];
        self.nor.conduct_into(&[*a, *b], &mut nor_output)?;
        self.not.conduct_into(&nor_output, outputs)
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
    const INPUTS: usize = 2;
    const OUTPUTS: usize = 1;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let mut nand1_output = [Signal::HighImpedance];
        self.nand1.conduct_into(&[*a, *b], &mut nand1_output)?;
        let mut nand2_output = [Signal::HighImpedance];
        self.nand2
            .conduct_into(&[*a, nand1_output[0]], &mut nand2_output)?;
        let mut nand3_output = [Signal::HighImpedance];
        self.nand3
            .conduct_into(&[*b, nand1_output[0]], &mut nand3_output)?;

        self.nand4
            .conduct_into(&[nand2_output[0], nand3_output[0]], outputs)
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
    const INPUTS: usize = 2;
    const OUTPUTS: usize = 1;

    fn conduct_into(
        &self,
        inputs: &[Signal],
        outputs: &mut [Signal],
    ) -> Result<(), HardwareError> {
        let [a, b] = inputs else {
            return Err(HardwareError::InvalidInputCount {
                expected: 2,
                actual: inputs.len(),
            });
        };

        let mut xor_output = [Signal::HighImpedance];
        self.xor.conduct_into(&[*a, *b], &mut xor_output)?;
        self.not.conduct_into(&xor_output, outputs)
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
