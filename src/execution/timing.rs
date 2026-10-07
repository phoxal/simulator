//! Monotonic active-wall accounting, independent of logical/native clock ownership.
use std::{collections::VecDeque, time::Duration};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum Pacing {
    #[default]
    Realtime,
    Fast,
}

impl Pacing {
    pub(crate) const fn label(self) -> &'static str {
        match self {
            Self::Realtime => "Realtime",
            Self::Fast => "Fast",
        }
    }
}

pub(super) struct Timing {
    pub(super) mode: Pacing,
    accumulated: Duration,
    started: Option<Duration>,
    anchor_wall: Duration,
    anchor_sim: Duration,
    recent: VecDeque<(Duration, Duration)>,
}

impl Timing {
    pub(super) fn new(mode: Pacing) -> Self {
        Self {
            mode,
            accumulated: Duration::ZERO,
            started: None,
            anchor_wall: Duration::ZERO,
            anchor_sim: Duration::ZERO,
            recent: VecDeque::new(),
        }
    }

    pub(super) fn wall(&self, now: Duration) -> Duration {
        self.accumulated + self.started.map_or(Duration::ZERO, |start| now - start)
    }

    pub(super) fn resume(&mut self, now: Duration, sim: Duration) {
        if self.started.is_none() {
            self.started = Some(now);
            self.reanchor(now, sim);
        }
    }

    pub(super) fn pause(&mut self, now: Duration) {
        self.accumulated = self.wall(now);
        self.started = None;
        self.recent.clear();
    }

    pub(super) fn set_mode(&mut self, mode: Pacing, now: Duration, sim: Duration) {
        if self.mode != mode {
            self.mode = mode;
            self.reanchor(now, sim);
        }
    }

    fn reanchor(&mut self, now: Duration, sim: Duration) {
        self.anchor_wall = self.wall(now);
        self.anchor_sim = sim;
        self.recent.clear();
        self.recent.push_back((self.anchor_wall, sim));
    }

    /// Delay after completed work; slow hosts never owe additional sleep.
    pub(super) fn delay(&self, now: Duration, sim: Duration) -> Duration {
        if self.mode == Pacing::Fast || self.started.is_none() {
            return Duration::ZERO;
        }
        (sim.saturating_sub(self.anchor_sim))
            .saturating_sub(self.wall(now).saturating_sub(self.anchor_wall))
    }

    /// Two active-wall seconds, sampled at most ten times per second.
    /// Initial (<100ms), paused and single-step displays have no rate.
    pub(super) fn speed(&mut self, now: Duration, sim: Duration) -> Option<f64> {
        self.started?;
        let wall = self.wall(now);
        if self
            .recent
            .back()
            .is_none_or(|(last, _)| wall - *last >= Duration::from_millis(100))
        {
            self.recent.push_back((wall, sim));
        }
        while self.recent.len() > 1
            && self
                .recent
                .get(1)
                .is_some_and(|(old, _)| wall - *old >= Duration::from_secs(2))
        {
            self.recent.pop_front();
        }
        let (old_wall, old_sim) = self.recent.front()?;
        let elapsed = wall - *old_wall;
        (elapsed >= Duration::from_millis(100))
            .then(|| sim.saturating_sub(*old_sim).as_secs_f64() / elapsed.as_secs_f64())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn deadlines_pacing_and_slow_host() {
        let mut timing = Timing::new(Pacing::Realtime);
        timing.resume(ms(1000), ms(0));
        assert_eq!(timing.delay(ms(1002), ms(10)), ms(8));
        assert_eq!(timing.delay(ms(1015), ms(10)), ms(0));
        timing.set_mode(Pacing::Fast, ms(1015), ms(10));
        assert_eq!(timing.delay(ms(1016), ms(1000)), ms(0));
    }

    #[test]
    fn pause_resume_and_mode_change_discard_debt_but_keep_active_wall() {
        let mut timing = Timing::new(Pacing::Realtime);
        timing.resume(ms(0), ms(0));
        timing.pause(ms(4));
        assert_eq!(timing.wall(ms(9000)), ms(4));
        timing.resume(ms(9000), ms(10));
        assert_eq!(timing.delay(ms(9000), ms(10)), ms(0));
        assert_eq!(timing.delay(ms(9002), ms(20)), ms(8));
        timing.set_mode(Pacing::Fast, ms(9002), ms(20));
        timing.set_mode(Pacing::Realtime, ms(9500), ms(5000));
        assert_eq!(timing.delay(ms(9500), ms(5000)), ms(0));
        assert_eq!(timing.wall(ms(9500)), ms(504));
    }

    #[test]
    fn paused_step_counts_only_work_and_reset_clears_counters() {
        let mut timing = Timing::new(Pacing::Realtime);
        timing.resume(ms(10000), ms(0));
        timing.pause(ms(10017));
        assert_eq!(timing.wall(ms(20000)), ms(17));
        assert_eq!(timing.speed(ms(20000), ms(10)), None);
        timing = Timing::new(timing.mode);
        assert_eq!(timing.wall(ms(30000)), ms(0));
    }

    #[test]
    fn recent_speed_exposes_slowdown_and_storage_is_bounded() {
        let mut timing = Timing::new(Pacing::Fast);
        timing.resume(ms(0), ms(0));
        assert_eq!(timing.speed(ms(50), ms(50)), None);
        for n in 1..=1000 {
            timing.speed(ms(n * 100), ms(n * 100));
        }
        assert_eq!(timing.speed(ms(100000), ms(100000)), Some(1.0));
        for n in 1..=30 {
            timing.speed(ms(100000 + n * 100), ms(100000 + n * 10));
        }
        let rate = timing.speed(ms(103000), ms(100300)).unwrap();
        assert!((rate - 0.1).abs() < 1e-10);
        assert!(timing.recent.len() <= 22);
        timing.pause(ms(103000));
        timing.resume(ms(900000), ms(100300));
        assert_eq!(timing.speed(ms(900000), ms(100300)), None);
    }
}
