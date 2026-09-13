//! Deterministic source cadence, expressed without an accumulating float clock.

use crate::remote::NativeProviderError;

const TICKS_PER_SECOND: u128 = 1_000_000_000_000_000;

#[derive(Clone, Copy, Debug)]
pub(crate) struct Cadence {
    ticks_per_quantum: u128,
    rate_microhertz: u64,
}

impl Cadence {
    pub(crate) fn new(rate_hz: f64, quantum_ns: u64) -> Result<Self, NativeProviderError> {
        let microhertz = rate_hz * 1_000_000.0;
        if !microhertz.is_finite()
            || microhertz < 1.0
            || microhertz > TICKS_PER_SECOND as f64
            || (microhertz - microhertz.round()).abs() > 1e-5
            || quantum_ns == 0
        {
            return Err(NativeProviderError::InvalidPayload(
                "publish rate must be positive with at most six decimal places".into(),
            ));
        }
        Self::from_microhertz(microhertz.round() as u64, quantum_ns)
    }

    pub(crate) fn from_microhertz(
        rate_microhertz: u64,
        quantum_ns: u64,
    ) -> Result<Self, NativeProviderError> {
        let ticks_per_quantum = u128::from(rate_microhertz) * u128::from(quantum_ns);
        if ticks_per_quantum == 0 || ticks_per_quantum > TICKS_PER_SECOND {
            return Err(NativeProviderError::InvalidPayload(
                "publish rate exceeds the scene quantum".into(),
            ));
        }
        Ok(Self {
            ticks_per_quantum,
            rate_microhertz,
        })
    }

    pub(crate) const fn rate_microhertz(self) -> u64 {
        self.rate_microhertz
    }

    pub(super) fn due(self, boundary: u64) -> bool {
        boundary == 0
            || u128::from(boundary) * self.ticks_per_quantum / TICKS_PER_SECOND
                > u128::from(boundary - 1) * self.ticks_per_quantum / TICKS_PER_SECOND
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn thirty_hertz_on_two_millisecond_quanta_has_no_accumulated_drift() {
        let cadence = Cadence::new(30.0, 2_000_000).unwrap();
        assert!(cadence.due(0));
        assert!(!cadence.due(16));
        assert!(cadence.due(17));
        assert!(!cadence.due(33));
        assert!(cadence.due(34));
        assert!(cadence.due(50));
        assert_eq!(
            (1..=500_000)
                .filter(|boundary| cadence.due(*boundary))
                .count(),
            30_000
        );
    }
    #[test]
    fn impossible_and_non_finite_rates_are_rejected() {
        for rate in [0.0, -1.0, 501.0, f64::INFINITY, f64::NAN] {
            assert!(Cadence::new(rate, 2_000_000).is_err());
        }
    }
}
