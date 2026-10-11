//! Bounded gestures fenced by execution, with wall-clock UI liveness only.
use super::{Run, viewport::Viewport};
use crate::mujoco::DRAG_LIVENESS as LIVENESS;
use crate::{app::RunState, authority::SimulationTransport};
use std::{
    sync::{Arc, Mutex},
    time::Instant,
};
fn expired(renewed: Instant, now: Instant) -> bool {
    now.saturating_duration_since(renewed) >= LIVENESS
}
pub(crate) trait DragTarget {
    fn begin(
        &mut self,
        body: usize,
        anchor: [f64; 3],
        camera: crate::mujoco::ViewCamera,
        paused: bool,
    ) -> Result<(), String>;
    fn update(&mut self, delta: [f64; 2], paused: bool) -> Result<(), String>;
    fn renew(&mut self, received: Instant);
    fn cancel(&mut self);
}
impl<T: SimulationTransport> DragTarget for Run<T> {
    fn begin(
        &mut self,
        body: usize,
        anchor: [f64; 3],
        camera: crate::mujoco::ViewCamera,
        paused: bool,
    ) -> Result<(), String> {
        self.begin_drag(body, anchor, camera, paused)
    }
    fn update(&mut self, delta: [f64; 2], paused: bool) -> Result<(), String> {
        self.update_drag(delta, paused)
    }
    fn renew(&mut self, received: Instant) {
        self.renew_drag(received)
    }
    fn cancel(&mut self) {
        self.cancel_drag()
    }
}
impl DragTarget for crate::mujoco::Scene {
    fn begin(
        &mut self,
        body: usize,
        anchor: [f64; 3],
        camera: crate::mujoco::ViewCamera,
        paused: bool,
    ) -> Result<(), String> {
        self.drag_begin(body, anchor, camera, paused)
    }
    fn update(&mut self, delta: [f64; 2], paused: bool) -> Result<(), String> {
        self.drag_update(delta, paused)
    }
    fn renew(&mut self, received: Instant) {
        self.drag_renew(received)
    }
    fn cancel(&mut self) {
        self.drag_cancel()
    }
}
#[derive(Default)]
pub(crate) struct Interaction {
    active: Option<(u64, crate::app::scene::SceneEpoch, Instant)>,
    closed: u64,
    begun: u64,
}
impl Interaction {
    pub fn terminal(&self) -> u64 {
        self.closed.max(self.begun)
    }
    pub fn cancel<T: DragTarget>(
        &mut self,
        run: &mut T,
        display: &Arc<Mutex<RunState>>,
    ) -> Result<(), String> {
        let mut state = display.lock().map_err(|_| "desktop state lock poisoned")?;
        if let Some((id, _, _)) = self.active.take() {
            self.closed = self.closed.max(id);
        }
        if let Some(begin) = &state.requests.drag_mailbox.begin {
            self.closed = self.closed.max(begin.id);
        }
        state.requests.drag_mailbox.end(self.closed);
        state.dragging = false;
        run.cancel();
        Ok(())
    }
    pub fn input<T: DragTarget>(
        &mut self,
        run: &mut T,
        viewport: &mut Viewport,
        display: &Arc<Mutex<RunState>>,
        paused: bool,
    ) -> Result<bool, String> {
        let (begin, update, ended, presented) = {
            let mut state = display.lock().map_err(|_| "desktop state lock poisoned")?;
            (
                state.requests.drag_mailbox.begin.take(),
                state.requests.drag_mailbox.update.take(),
                state.requests.drag_mailbox.ended,
                state.requests.presented.clone(),
            )
        };
        self.closed = self.closed.max(ended);
        if self
            .active
            .as_ref()
            .is_some_and(|(id, _, renewed)| *id <= self.closed || expired(*renewed, Instant::now()))
        {
            self.cancel(run, display)?;
        }
        let mut changed = false;
        let result = (|| {
            if let Some(begin) = begin {
                if begin.id <= self.closed
                    || begin.id <= self.begun
                    || expired(begin.received, Instant::now())
                {
                    return Ok(());
                }
                self.begun = begin.id;
                if let Some((id, _, _)) = self.active.take() {
                    self.closed = self.closed.max(id);
                }
                run.cancel();
                let view = presented.as_ref().ok_or("drag has no presented frame")?;
                let (body, anchor, camera) = viewport.drag_anchor(&begin, view)?;
                run.begin(body, anchor, camera, paused)?;
                run.renew(begin.received);
                self.active = Some((begin.id, begin.frame.epoch, begin.received));
                changed = true;
            }
            if let Some(update) = update
                && let Some((id, epoch, renewed)) = &mut self.active
                && update.id == *id
                && update.epoch == *epoch
                && !expired(update.received, Instant::now())
                && update.received >= *renewed
            {
                run.update(update.delta, paused)?;
                run.renew(update.received);
                *renewed = update.received;
                changed = true;
            }
            Ok::<(), String>(())
        })();
        if let Err(error) = result {
            changed = true;
            self.cancel(run, display)?;
            display
                .lock()
                .map_err(|_| "desktop state lock poisoned")?
                .interaction_error = Some(error);
        } else if changed {
            display
                .lock()
                .map_err(|_| "desktop state lock poisoned")?
                .interaction_error = None;
        }
        display
            .lock()
            .map_err(|_| "desktop state lock poisoned")?
            .dragging = self.active.is_some();
        Ok(changed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    #[test]
    fn held_renewal_and_expiry_use_wall_liveness_independent_of_logical_time() {
        let start = Instant::now();
        assert!(!expired(start, start + Duration::from_millis(249)));
        assert!(expired(start, start + Duration::from_millis(250)));
        let stationary_renewal = start + Duration::from_millis(200);
        assert!(!expired(
            stationary_renewal,
            start + Duration::from_millis(400)
        ));
        assert!(expired(
            stationary_renewal,
            start + Duration::from_millis(450)
        ));
        assert!(!expired(start, start));
    }
}
