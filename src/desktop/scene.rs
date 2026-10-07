//! Immutable presentation identity and bounded, replaceable scene requests.
use crate::mujoco::{ModelIdentity, RenderedCamera, StateSnapshot, ViewCamera};
use std::sync::Arc;

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct SceneEpoch {
    pub execution: String,
    pub model: ModelIdentity,
    pub generation: u64,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct FrameIdentity {
    pub epoch: SceneEpoch,
    pub serial: u64,
}
#[derive(Clone, Debug)]
pub(crate) struct PresentedView {
    pub identity: FrameIdentity,
    pub camera: ViewCamera,
    pub resolution: [usize; 2],
    pub snapshot: StateSnapshot,
}
pub(crate) struct ViewportFrame {
    pub image: RenderedCamera,
    pub view: Arc<PresentedView>,
}
#[derive(Clone, Debug)]
pub(crate) struct CameraRequest {
    pub epoch: SceneEpoch,
    pub camera: ViewCamera,
}
#[derive(Clone, Copy, Debug)]
pub(crate) enum SceneAction {
    Pick([f64; 2]),
    SelectBody(usize),
    Clear,
    Focus,
    DefaultCamera,
}
#[derive(Clone, Debug)]
pub(crate) struct SceneRequest {
    pub frame: FrameIdentity,
    pub action: SceneAction,
}
#[derive(Clone, Debug)]
pub(crate) struct SceneResult {
    pub frame: FrameIdentity,
    pub stale: bool,
    pub camera: Option<ViewCamera>,
}

impl SceneRequest {
    pub fn matches(
        &self,
        epoch: &SceneEpoch,
        presented: &PresentedView,
        camera: ViewCamera,
    ) -> bool {
        self.frame.epoch == *epoch && self.frame == presented.identity && presented.camera == camera
    }
}

/// Two replaceable slots plus a terminal high-water mark. End cannot be lost
/// under update flooding, and its gesture cannot be reopened.
#[derive(Default)]
pub(crate) struct DragMailbox {
    pub begin: Option<DragBegin>,
    pub update: Option<DragUpdate>,
    pub ended: u64,
}
#[derive(Clone)]
pub(crate) struct DragBegin {
    pub id: u64,
    pub frame: FrameIdentity,
    pub xy: [f64; 2],
    pub received: std::time::Instant,
}
#[derive(Clone)]
pub(crate) struct DragUpdate {
    pub id: u64,
    pub epoch: SceneEpoch,
    pub delta: [f64; 2],
    pub received: std::time::Instant,
}
impl DragMailbox {
    pub fn end(&mut self, id: u64) {
        self.ended = self.ended.max(id);
        if self.begin.as_ref().is_some_and(|x| x.id <= self.ended) {
            self.begin = None;
        }
        if self.update.as_ref().is_some_and(|x| x.id <= self.ended) {
            self.update = None;
        }
    }
}

#[cfg(test)]
mod drag_tests {
    use super::*;
    #[test]
    fn terminal_high_water_dominates_flooded_updates_and_pending_begin() {
        let epoch = SceneEpoch {
            execution: "execution".into(),
            model: crate::mujoco::ModelIdentity([7; 32]),
            generation: 1,
        };
        let mut mailbox = DragMailbox::default();
        for id in 1..=10000 {
            mailbox.begin = Some(DragBegin {
                id,
                frame: FrameIdentity {
                    epoch: epoch.clone(),
                    serial: id,
                },
                xy: [0.5; 2],
                received: std::time::Instant::now(),
            });
            mailbox.update = Some(DragUpdate {
                id,
                epoch: epoch.clone(),
                delta: [0.0; 2],
                received: std::time::Instant::now(),
            });
        }
        assert_eq!(mailbox.begin.as_ref().unwrap().id, 10000);
        assert_eq!(mailbox.update.as_ref().unwrap().id, 10000);
        mailbox.end(10000);
        assert!(mailbox.begin.is_none() && mailbox.update.is_none());
        mailbox.end(1);
        assert_eq!(mailbox.ended, 10000);
        // Even a stale update queued after end retains the terminal fence.
        mailbox.update = Some(DragUpdate {
            id: 1,
            epoch,
            delta: [1.0; 2],
            received: std::time::Instant::now(),
        });
        assert!(mailbox.update.as_ref().unwrap().id <= mailbox.ended);
    }
}
