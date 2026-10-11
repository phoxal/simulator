//! Application-owned bounded requests and one-way worker observations.
pub(crate) mod scene;
pub(crate) mod world;
#[cfg(test)]
use crate::config::Options;
use crate::execution::Pacing;
use crate::mujoco::{NativeBody, NativeSelection, ViewCamera};
use crate::notice::Notice;
use scene::{CameraRequest, PresentedView, SceneRequest, SceneResult, ViewportFrame};
use std::sync::{Arc, Mutex};
#[cfg(test)]
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};

#[derive(Clone, Copy, Debug)]
pub(super) enum Command {
    Run,
    Pause,
    Step,
    Reset,
    Stop,
    SetPacing(Pacing),
}

#[derive(Default)]
pub(super) struct RunSnapshot {
    pub(super) document: world::DocumentView,
    pub(super) native_state: Option<Arc<crate::mujoco::StateSnapshot>>,
    pub(super) robot_name: String,
    pub(super) robot_root: Option<usize>,
    pub(super) boundary: u64,
    pub(super) time_seconds: f64,
    pub(super) wall_seconds: f64,
    pub(super) speed: Option<f64>,
    pub(super) pacing: Pacing,
    pub(super) generation: u64,
    pub(super) running: bool,
    pub(super) ready: bool,
    pub(super) finished: bool,
    pub(super) completion: Option<Completion>,
    pub(super) close_requested: bool,
    pub(super) error: Option<Notice>,
    pub(super) document_error: Option<Notice>,
    pub(super) phase: RunStatus,
    pub(super) stopping: bool,
    pub(super) cleanup_failed: bool,
    pub(super) frame: Option<ViewportFrame>,
    pub(super) camera: Option<ViewCamera>,
    pub(super) scene_result: Option<SceneResult>,
    pub(super) dragging: bool,
    pub(super) interaction_error: Option<String>,
    pub(super) bodies: Arc<[NativeBody]>,
    pub(super) selection: Option<NativeSelection>,
    pub(super) controls: Vec<f64>,
    pub(super) actuation_boundary: Option<u64>,
    pub(super) actuation_products: usize,
    #[cfg(test)]
    pub(super) qualification_cuts: std::collections::VecDeque<serde_json::Value>,
}

#[derive(Default)]
pub(super) struct RunRequests {
    pub(super) pending_camera: Option<CameraRequest>,
    pub(super) presented: Option<Arc<PresentedView>>,
    pub(super) pending_scene: Option<SceneRequest>,
    /// Desired native viewport size in interface points, replaceable each
    /// frame; the worker fits the render resolution within the model
    /// framebuffer and pixel budget and ignores intermediate resize chatter.
    pub(super) viewport_size: Option<[f32; 2]>,
    pub(super) drag_mailbox: scene::DragMailbox,
}

#[derive(Default)]
pub(super) struct RunState {
    pub(super) snapshot: RunSnapshot,
    pub(super) requests: RunRequests,
}
impl std::ops::Deref for RunState {
    type Target = RunSnapshot;
    fn deref(&self) -> &Self::Target {
        &self.snapshot
    }
}
impl std::ops::DerefMut for RunState {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.snapshot
    }
}
pub(super) struct Worker {
    pub(super) commands: Receiver<Command>,
    pub(super) display: Arc<Mutex<RunState>>,
    pub(super) cancel: crate::cancellation::Cancellation,
}

impl RunState {
    pub(super) fn observe(&mut self) -> Self {
        Self {
            snapshot: RunSnapshot {
                document: self.document.clone(),
                native_state: self.native_state.clone(),
                robot_name: self.robot_name.clone(),
                robot_root: self.robot_root,
                boundary: self.boundary,
                time_seconds: self.time_seconds,
                wall_seconds: self.wall_seconds,
                speed: self.speed,
                pacing: self.pacing,
                generation: self.generation,
                running: self.running,
                ready: self.ready,
                finished: self.finished,
                completion: self.completion,
                close_requested: self.close_requested,
                error: self.error.clone(),
                document_error: self.document_error.clone(),
                phase: self.phase,
                stopping: self.stopping,
                cleanup_failed: self.cleanup_failed,
                frame: self.frame.take(),
                camera: self.camera,
                scene_result: self.scene_result.take(),
                dragging: self.dragging,
                interaction_error: self.interaction_error.clone(),
                bodies: self.bodies.clone(),
                selection: self.selection.clone(),
                controls: self.controls.clone(),
                actuation_boundary: self.actuation_boundary,
                actuation_products: self.actuation_products,
                #[cfg(test)]
                qualification_cuts: Default::default(),
            },
            requests: RunRequests {
                presented: self.requests.presented.clone(),
                drag_mailbox: scene::DragMailbox {
                    ended: self.requests.drag_mailbox.ended,
                    ..Default::default()
                },
                ..Default::default()
            },
        }
    }

    pub(super) fn request(&mut self, generation: u64, mut requests: RunRequests) {
        // Only requests cross this boundary. Observed outputs are never merged back.
        if !self.ready || self.stopping || self.generation != generation {
            return;
        }
        let Some(presented) = requests.presented.take() else {
            return;
        };
        if self
            .frame
            .as_ref()
            .is_some_and(|frame| frame.view.identity.epoch != presented.identity.epoch)
            || self
                .requests
                .presented
                .as_ref()
                .is_some_and(|view| view.identity.epoch != presented.identity.epoch)
        {
            return;
        }
        self.requests.presented = Some(presented);
        if requests.pending_camera.is_some() {
            self.requests.pending_camera = requests.pending_camera;
        }
        if requests.pending_scene.is_some() {
            self.requests.pending_scene = requests.pending_scene;
        }
        if requests.viewport_size.is_some() {
            self.requests.viewport_size = requests.viewport_size;
        }
        self.requests.drag_mailbox.end(requests.drag_mailbox.ended);
        if let Some(begin) = requests.drag_mailbox.begin
            && begin.id > self.requests.drag_mailbox.ended
        {
            self.requests.drag_mailbox.begin = Some(begin);
        }
        if let Some(update) = requests.drag_mailbox.update
            && update.id > self.requests.drag_mailbox.ended
        {
            self.requests.drag_mailbox.update = Some(update);
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum Completion {
    Idle,
    Probed,
    Cancelled,
    Completed,
    Stopped,
    Failed,
}

pub(super) enum RunOutcome {
    Idle,
    Probed,
    Cancelled,
    Executed {
        completion: Completion,
        evidence: Box<crate::runtime::TerminalEvidence>,
    },
}
impl RunOutcome {
    pub(super) fn terminal(&self) -> Option<&crate::runtime::TerminalEvidence> {
        match self {
            Self::Executed { evidence, .. } => Some(evidence),
            _ => None,
        }
    }
    pub(super) fn completion(&self) -> Completion {
        match self {
            Self::Idle => Completion::Idle,
            Self::Probed => Completion::Probed,
            Self::Cancelled => Completion::Cancelled,
            Self::Executed { completion, .. } => *completion,
        }
    }
}
pub(super) struct RunHandle {
    pub(super) commands: Sender<Command>,
    pub(super) cancel: crate::cancellation::Cancellation,
}

impl RunHandle {
    pub(super) fn send(&self, command: Command) -> Result<(), String> {
        if matches!(command, Command::Stop) {
            self.cancel.cancel();
            return Ok(());
        }
        if self.cancel.is_cancelled() {
            return Err("Stopping; no new command admitted".into());
        }
        self.commands
            .try_send(command)
            .map_err(|error| format!("Control unavailable: {error}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_queue_reports_rejection_but_stop_latches_and_fences_new_commands() {
        let (commands, mut receiver) = mpsc::channel(8);
        let handle = RunHandle {
            commands,
            cancel: Default::default(),
        };
        for _ in 0..8 {
            handle.send(Command::Step).unwrap();
        }
        assert!(
            handle
                .send(Command::Reset)
                .unwrap_err()
                .contains("Control unavailable")
        );
        handle.send(Command::Stop).unwrap();
        assert!(handle.cancel.is_cancelled());
        assert!(handle.send(Command::Run).unwrap_err().contains("Stopping"));
        for _ in 0..8 {
            assert!(matches!(receiver.try_recv().unwrap(), Command::Step));
        }
        assert!(receiver.try_recv().is_err());
        handle.send(Command::Stop).unwrap();
    }

    #[test]
    fn completion_and_cleanup_failure_survive_observation_and_fence_restart() {
        let display = Arc::new(Mutex::new(RunState::default()));
        {
            let mut state = display.lock().unwrap();
            state.finished = true;
            state.completion = Some(Completion::Failed);
            state.cleanup_failed = true;
            state.error = Some(Notice::from("authority release unconfirmed"));
            for boundary in [4, 5, 6] {
                state.boundary = boundary;
                let seen = state.observe();
                assert_eq!(seen.completion, Some(Completion::Failed));
                assert!(seen.cleanup_failed);
                assert!(
                    seen.error
                        .as_ref()
                        .unwrap()
                        .to_string()
                        .contains("authority release")
                );
            }
        }
        let options = Options {
            scene: "unused.xml".into(),
            bundle: "unused".into(),
            simulation_run: None,
            probe: false,
            json: false,
            presentation: crate::config::Presentation::Desktop,
            scope: None,
            connect: None,
            supervisor_id: None,
            run_id: None,
            bound: None,
            auto_run: false,
        };
        assert!(
            world::Handle::qualification(options, display.clone())
                .err()
                .unwrap()
                .contains("cleanup is unconfirmed")
        );
        assert_eq!(display.lock().unwrap().completion, Some(Completion::Failed));
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(super) enum RunStatus {
    #[default]
    Idle,
    CheckingEngine,
    Preparing,
    StartingSupervisor,
    WaitingForSupervisor,
    Connecting,
    Executing,
    Stopping,
}
impl RunStatus {
    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Idle | Self::Executing => "",
            Self::CheckingEngine => "Checking runtime",
            Self::Preparing => "Preparing scene",
            Self::StartingSupervisor => "Starting supervisor",
            Self::WaitingForSupervisor => "Waiting for supervisor",
            Self::Connecting => "Connecting",
            Self::Stopping => "Stopping",
        }
    }
}
