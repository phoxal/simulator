//! Application world and document owner. Native work never lives in widgets.
use super::{Command, Completion, RunHandle, RunState, Worker};
use crate::document::{Document, RobotSource};
use crate::execution::viewport::Viewport;
use crate::mujoco::{Model, Scene, StateSnapshot};
use crate::runtime::PreparedRun;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::sync::mpsc;

#[derive(Clone, Default)]
pub(crate) struct DocumentView {
    pub scene_name: String,
    pub robot: Option<String>,
    pub attachments: Vec<(String, [f64; 3], [f64; 4])>,
    pub candidate: Option<CandidateView>,
    pub removed_robot: Option<String>,
    pub busy: bool,
    pub cancelable: bool,
}
#[derive(Clone)]
pub(crate) struct CandidateView {
    pub identity: String,
    pub build: PathBuf,
    pub target: String,
    pub position: [f64; 3],
    pub quaternion: [f64; 4],
    pub image: Arc<crate::mujoco::RenderedCamera>,
}
#[derive(Clone)]
pub(crate) enum Edit {
    New,
    Scene(PathBuf),
    Import {
        build: PathBuf,
        target: String,
        position: [f64; 3],
        quaternion: [f64; 4],
    },
    Place,
    CancelImport,
    Remove,
    Undo,
}
enum Request {
    Edit {
        edit: Edit,
        cancel: crate::cancellation::Cancellation,
    },
    Start {
        command: Command,
        commands: mpsc::Receiver<Command>,
        cancel: crate::cancellation::Cancellation,
    },
    Shutdown,
}
// Signal handlers outlive individual executions, including an ended worker.
// Keep their receiver/driver alive until the application Handle is shut down.
struct ApplicationSignals {
    stop: crate::cancellation::Cancellation,
    _runtime: tokio::runtime::Runtime,
}
impl ApplicationSignals {
    fn new(
        display: Arc<Mutex<RunState>>,
        control: Arc<Mutex<RunHandle>>,
        quit: crate::cancellation::Cancellation,
    ) -> Result<Self, String> {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()
            .map_err(|e| e.to_string())?;
        let (mut interrupt, mut terminate) = {
            let _entered = runtime.enter();
            (
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())
                    .map_err(|e| e.to_string())?,
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
                    .map_err(|e| e.to_string())?,
            )
        };
        let stop = crate::cancellation::Cancellation::default();
        let ending = stop.clone();
        runtime.spawn(async move {
            loop {
                tokio::select! {
                    biased;
                    _ = ending.wait() => break,
                    _ = interrupt.recv() => {},
                    _ = terminate.recv() => {},
                }
                if let Ok(mut state) = display.lock()
                    && !state.close_requested
                {
                    eprintln!("phoxal-simulator: application termination received");
                    state.close_requested = true;
                }
                quit.cancel();
                if let Ok(control) = control.lock() {
                    control.cancel.cancel();
                }
            }
        });
        Ok(Self {
            stop,
            _runtime: runtime,
        })
    }
}
impl Drop for ApplicationSignals {
    fn drop(&mut self) {
        self.stop.cancel();
    }
}
pub(crate) struct Handle {
    requests: mpsc::Sender<Request>,
    pub control: Arc<Mutex<RunHandle>>,
    pub display: Arc<Mutex<RunState>>,
    admission: Mutex<crate::cancellation::Cancellation>,
    quit: crate::cancellation::Cancellation,
    thread: Option<std::thread::JoinHandle<Result<super::RunOutcome, String>>>,
    signals: Option<ApplicationSignals>,
}
impl Handle {
    #[cfg(test)]
    pub(crate) fn fixture(display: Arc<Mutex<RunState>>) -> Self {
        let (requests, _) = mpsc::channel(8);
        let (commands, _) = mpsc::channel(8);
        Self {
            requests,
            control: Arc::new(Mutex::new(RunHandle {
                commands,
                cancel: Default::default(),
            })),
            display,
            admission: Mutex::new(Default::default()),
            quit: Default::default(),
            thread: None,
            signals: None,
        }
    }
    pub fn start(options: Option<crate::config::Options>) -> Result<Self, String> {
        Self::spawn(options, Arc::new(Mutex::new(RunState::default())))
    }
    #[cfg(test)]
    pub(crate) fn qualification(
        options: crate::config::Options,
        display: Arc<Mutex<RunState>>,
    ) -> Result<Self, String> {
        {
            let mut state = display.lock().map_err(|_| "World state lock poisoned")?;
            if state.cleanup_failed {
                return Err("Previous cleanup is unconfirmed; restart is disabled".into());
            }
            if state.stopping || (state.ready && !state.finished) {
                return Err("Previous world is still active".into());
            }
            *state = RunState::default();
        }
        Self::spawn(Some(options), display)
    }
    fn spawn(
        options: Option<crate::config::Options>,
        display: Arc<Mutex<RunState>>,
    ) -> Result<Self, String> {
        let (requests, request_receiver) = mpsc::channel(8);
        let (commands, receiver) = mpsc::channel(8);
        let cancel = crate::cancellation::Cancellation::default();
        if let Some(initial) = &options {
            let command = if initial.auto_run {
                Command::Run
            } else {
                Command::Pause
            };
            commands.try_send(command).map_err(|e| e.to_string())?;
            requests
                .try_send(Request::Start {
                    command,
                    commands: receiver,
                    cancel: cancel.clone(),
                })
                .map_err(|e| e.to_string())?;
        }
        let control = Arc::new(Mutex::new(RunHandle { commands, cancel }));
        let quit = crate::cancellation::Cancellation::default();
        let signals = ApplicationSignals::new(display.clone(), control.clone(), quit.clone())?;
        let state = display.clone();
        let shutdown = quit.clone();
        let thread = std::thread::Builder::new()
            .name("simulation-world".into())
            .spawn(move || {
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    run(options, request_receiver, &state, shutdown)
                }))
                .unwrap_or_else(|_| {
                    state.lock().unwrap().cleanup_failed = true;
                    Err("Native world worker panicked; cleanup is unconfirmed".into())
                });
                if let Err(e) = &result {
                    let mut s = state.lock().unwrap();
                    s.error = Some(e.clone().into());
                    s.finished = true;
                    s.document.busy = false;
                }
                result
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            requests,
            control,
            display,
            admission: Mutex::new(Default::default()),
            quit,
            signals: Some(signals),
            thread: Some(thread),
        })
    }
    pub fn edit(&self, edit: Edit) -> Result<(), String> {
        let mut s = self
            .display
            .lock()
            .map_err(|_| "World state lock poisoned")?;
        if s.cleanup_failed || !s.finished || s.stopping || s.document.busy {
            return Err(
                "Confirm Stop and successful cleanup before editing this simulation".into(),
            );
        }
        let cancelable = matches!(edit, Edit::New | Edit::Scene(_) | Edit::Import { .. });
        let cancel = crate::cancellation::Cancellation::default();
        self.requests
            .try_send(Request::Edit {
                edit,
                cancel: cancel.clone(),
            })
            .map_err(|e| e.to_string())?;
        *self
            .admission
            .lock()
            .map_err(|_| "Admission lock poisoned")? = cancel;
        s.document.busy = true;
        s.document.cancelable = cancelable;
        Ok(())
    }
    pub fn cancel_admission(&self) {
        if let Ok(cancel) = self.admission.lock() {
            cancel.cancel();
        }
    }
    pub fn send(&self, command: Command) -> Result<(), String> {
        let mut s = self
            .display
            .lock()
            .map_err(|_| "World state lock poisoned")?;
        if s.cleanup_failed {
            return Err("Cleanup is unconfirmed; execution is disabled".into());
        }
        if !s.finished || matches!(command, Command::Stop) {
            let result = self
                .control
                .lock()
                .map_err(|_| "Control lock poisoned")?
                .send(command);
            if matches!(command, Command::Stop) && !s.finished {
                s.stopping = true;
            }
            return result;
        }
        if s.document.busy {
            return Err("World admission is outstanding".into());
        }
        if !matches!(command, Command::Run | Command::Step | Command::Reset) {
            return Ok(());
        }
        let (sender, commands) = mpsc::channel(8);
        sender.try_send(command).map_err(|e| e.to_string())?;
        let cancel = crate::cancellation::Cancellation::default();
        self.requests
            .try_send(Request::Start {
                command,
                commands,
                cancel: cancel.clone(),
            })
            .map_err(|e| e.to_string())?;
        *self.control.lock().map_err(|_| "Control lock poisoned")? = RunHandle {
            commands: sender,
            cancel,
        };
        s.finished = false;
        s.ready = false;
        s.phase = super::RunStatus::Preparing;
        s.document.busy = true;
        Ok(())
    }
    pub fn shutdown(&mut self) -> Result<super::RunOutcome, String> {
        self.quit.cancel();
        self.control
            .lock()
            .map_err(|_| "Control lock poisoned")?
            .cancel
            .cancel();
        let _ = self.requests.try_send(Request::Shutdown);
        let outcome = self
            .thread
            .take()
            .map(|t| {
                eprintln!("phoxal-simulator: owned world join started");
                let outcome = t
                    .join()
                    .unwrap_or_else(|_| Err("World worker cleanup unconfirmed".into()));
                eprintln!(
                    "phoxal-simulator: owned world join returned ok={}",
                    outcome.is_ok()
                );
                outcome
            })
            .unwrap_or(Ok(super::RunOutcome::Idle));
        self.signals.take();
        outcome
    }
}
impl Drop for Handle {
    fn drop(&mut self) {
        if let Err(e) = self.shutdown() {
            eprintln!("phoxal-simulator: {e}");
        }
    }
}

struct Candidate {
    prepared: PreparedRun,
    source: RobotSource,
    view: CandidateView,
}
struct LaunchPreferences {
    scope: Option<String>,
    connect: Option<String>,
    supervisor_id: Option<String>,
    run_id: Option<String>,
    bound: Option<crate::config::Bound>,
    simulation_run: Option<PathBuf>,
}
impl From<crate::config::Options> for LaunchPreferences {
    fn from(options: crate::config::Options) -> Self {
        Self {
            scope: options.scope,
            connect: options.connect,
            supervisor_id: options.supervisor_id,
            run_id: options.run_id,
            bound: options.bound,
            simulation_run: options.simulation_run,
        }
    }
}
struct World {
    document: Document,
    scene: Scene,
    prepared: Option<PreparedRun>,
    candidate: Option<Candidate>,
    removed: Option<(PreparedRun, RobotSource)>,
    view: Viewport,
    frozen: Option<StateSnapshot>,
    generation: u64,
    serial: u64,
    launch: Option<LaunchPreferences>,
}
impl World {
    fn ready_view(scene: &Scene, execution: &str, generation: u64) -> Result<Viewport, String> {
        let mut view = Viewport::new(scene.model(), execution, generation)?;
        view.render(&scene.snapshot().map_err(|e| e.to_string())?)?;
        Ok(view)
    }
    fn admit(document: Document) -> Result<Self, String> {
        document.definition.scene.verify_live()?;
        let closed = document.definition.scene.close()?;
        crate::mujoco::validate_scene_policy(&closed).map_err(|e| e.to_string())?;
        let prepared = if let Some(source) = &document.definition.robot {
            let bundle = crate::bundle::BundleFacts::load(&source.build)?;
            if digest(&bundle.manifest_bytes) != source.manifest {
                return Err(
                    "Prepared build manifest changed; rebuild and import it again explicitly"
                        .into(),
                );
            }
            let model = crate::composition::compose_closed(
                closed.clone(),
                &bundle,
                &source.target,
                Some((source.position, source.quaternion)),
            )?;
            if model.identity().to_hex() != source.model {
                return Err(
                    "Prepared native model/resources changed; rebuild and import them again explicitly".into(),
                );
            }
            Some(crate::runtime::prepare_model(model, bundle)?)
        } else {
            None
        };
        let model = match &prepared {
            Some(p) => p.model().clone(),
            None => Model::from_closed(closed).map_err(|e| e.to_string())?,
        };
        let scene = Scene::new(model).map_err(|e| e.to_string())?;
        let view = Self::ready_view(&scene, "world-0", 0)?;
        Ok(Self {
            document,
            scene,
            prepared,
            candidate: None,
            removed: None,
            view,
            frozen: None,
            generation: 0,
            serial: 0,
            launch: None,
        })
    }
    fn snapshot(&self) -> Result<StateSnapshot, String> {
        self.frozen
            .clone()
            .map(Ok)
            .unwrap_or_else(|| self.scene.snapshot().map_err(|e| e.to_string()))
    }
    fn verify(&self) -> Result<(), String> {
        self.document.definition.scene.verify_live()?;
        if let Some(p) = &self.prepared {
            p.verify_sources()?;
        }
        Ok(())
    }
    fn edit(&mut self, edit: Edit) -> Result<(), String> {
        self.edit_checked(edit, &crate::cancellation::Cancellation::default())
    }
    fn edit_checked(
        &mut self,
        edit: Edit,
        cancel: &crate::cancellation::Cancellation,
    ) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("Scene change canceled".into());
        }
        match edit {
            Edit::New | Edit::Scene(_) => {
                let document = match edit {
                    Edit::New => Document::default_scene()?,
                    Edit::Scene(p) => Document::scene(&p)?,
                    _ => unreachable!(),
                };
                let mut candidate = Self::admit(document)?;
                candidate.serial = self.serial + 1;
                candidate.view =
                    Self::ready_view(&candidate.scene, &format!("world-{}", candidate.serial), 0)?;
                if cancel.is_cancelled() {
                    return Err("Scene change canceled".into());
                }
                *self = candidate;
            }
            Edit::Import {
                build,
                target,
                position,
                quaternion,
            } => {
                if self.prepared.is_some() {
                    return Err("Remove the current robot before importing another".into());
                }
                self.document.definition.scene.verify_live()?;
                if !self
                    .scene
                    .model()
                    .eligible_attachments()
                    .iter()
                    .any(|a| a.0 == target)
                {
                    return Err(
                        "Selected scene has no eligible authored world-level attachment".into(),
                    );
                }
                if !position
                    .iter()
                    .chain(quaternion.iter())
                    .all(|v| v.is_finite())
                    || (quaternion.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-8
                {
                    return Err("Placement must be finite with a unit quaternion".into());
                }
                let bundle = crate::bundle::BundleFacts::load(&build)?;
                let manifest = digest(&bundle.manifest_bytes);
                let build = bundle.root.clone();
                let model = crate::composition::compose_closed(
                    self.document.definition.scene.close()?,
                    &bundle,
                    &target,
                    Some((position, quaternion)),
                )?;
                let source = RobotSource {
                    build: build.clone(),
                    manifest,
                    model: model.identity().to_hex(),
                    target: target.clone(),
                    position,
                    quaternion,
                };
                let prepared = crate::runtime::prepare_model(model, bundle)?;
                let snapshot = Scene::new(prepared.model().clone())
                    .map_err(|e| e.to_string())?
                    .snapshot()
                    .map_err(|e| e.to_string())?;
                let mut preview = Viewport::new(prepared.model(), "candidate", 0)?;
                let image = Arc::new(preview.render(&snapshot)?.image);
                if cancel.is_cancelled() {
                    return Err("Robot preview canceled".into());
                }
                let identity = format!(
                    "{} · {}\nManifest {}\nModel {}",
                    prepared.bundle().robot_id,
                    prepared.bundle().target,
                    source.manifest,
                    source.model
                );
                self.candidate = Some(Candidate {
                    prepared,
                    source,
                    view: CandidateView {
                        identity,
                        build,
                        target,
                        position,
                        quaternion,
                        image,
                    },
                });
            }
            Edit::CancelImport => self.candidate = None,
            Edit::Place => {
                self.document.definition.scene.verify_live()?;
                let candidate = self
                    .candidate
                    .as_ref()
                    .ok_or("No validated placement preview")?;
                candidate.prepared.verify_sources()?;
                // Construct every native owner before changing the confirmed document.
                let scene =
                    Scene::new(candidate.prepared.model().clone()).map_err(|e| e.to_string())?;
                let mut view = Self::ready_view(&scene, &format!("world-{}", self.serial + 1), 0)?;
                view.select_body(
                    scene
                        .model()
                        .robot_root()
                        .ok_or("Admitted robot root is unavailable")?,
                    &scene.snapshot().map_err(|e| e.to_string())?,
                )?;
                let candidate = self.candidate.take().expect("validated candidate");
                self.prepared = Some(candidate.prepared);
                self.document.definition.robot = Some(candidate.source);
                self.scene = scene;
                self.view = view;
                self.frozen = None;
                self.serial += 1;
                self.generation = 0;
                self.removed = None;
                if let Some(launch) = &mut self.launch {
                    launch.simulation_run = None;
                    launch.connect = None;
                }
            }
            Edit::Remove => {
                if self.prepared.is_none() {
                    return Err("No robot is imported".into());
                }
                let model = Model::from_closed(self.document.definition.scene.close()?)
                    .map_err(|e| e.to_string())?;
                let scene = Scene::new(model.clone()).map_err(|e| e.to_string())?;
                let view = Self::ready_view(&scene, &format!("world-{}", self.serial + 1), 0)?;
                self.removed = Some((
                    self.prepared.take().unwrap(),
                    self.document.definition.robot.take().unwrap(),
                ));
                self.scene = scene;
                self.view = view;
                self.frozen = None;
                self.serial += 1;
                self.generation = 0;
                self.candidate = None;
                if let Some(launch) = &mut self.launch {
                    launch.simulation_run = None;
                    launch.connect = None;
                }
            }
            Edit::Undo => {
                let prepared = &self.removed.as_ref().ok_or("No removal to undo")?.0;
                prepared.verify_sources()?;
                let scene = Scene::new(prepared.model().clone()).map_err(|e| e.to_string())?;
                let view = Self::ready_view(&scene, &format!("world-{}", self.serial + 1), 0)?;
                let (prepared, source) = self.removed.take().unwrap();
                self.prepared = Some(prepared);
                self.document.definition.robot = Some(source);
                self.scene = scene;
                self.view = view;
                self.frozen = None;
                self.serial += 1;
                self.generation = 0;
            }
        }
        Ok(())
    }
    fn document_view(&self) -> DocumentView {
        DocumentView {
            scene_name: self
                .document
                .definition
                .scene
                .path
                .as_ref()
                .and_then(|p| p.file_name())
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_else(|| "Default scene".into()),
            robot: self.prepared.as_ref().map(|p| p.bundle().robot_id.clone()),
            attachments: self.scene.model().eligible_attachments(),
            candidate: self.candidate.as_ref().map(|c| c.view.clone()),
            removed_robot: self
                .removed
                .as_ref()
                .map(|(prepared, _)| prepared.bundle().robot_id.clone()),
            busy: false,
            cancelable: false,
        }
    }
    fn publish(
        &mut self,
        display: &Arc<Mutex<RunState>>,
        running: bool,
        active: bool,
        pacing: crate::execution::Pacing,
    ) -> Result<(), String> {
        let snapshot = self.snapshot()?;
        self.view.input(display)?;
        let frame = self.view.render(&snapshot)?;
        let mut state = display.lock().map_err(|_| "World state lock poisoned")?;
        if state
            .native_state
            .as_ref()
            .is_none_or(|s| s.model_identity() != snapshot.model_identity())
            || state
                .requests
                .presented
                .as_ref()
                .is_some_and(|view| view.identity.epoch != frame.view.identity.epoch)
        {
            state.requests = Default::default();
            state.bodies = Default::default();
            state.selection = None;
            state.scene_result = None;
        }
        state.document = self.document_view();
        state.frame = Some(frame);
        state.native_state = Some(Arc::new(snapshot.clone()));
        state.boundary = snapshot.boundary();
        state.time_seconds = snapshot.time_seconds();
        state.generation = self.generation;
        state.pacing = pacing;
        state.ready = true;
        state.finished = !active;
        state.running = running;
        state.stopping = false;
        self.view.publish(&mut state);
        Ok(())
    }
}
/// An admission refusal is scoped to the refused action: the current
/// simulation is kept, and the complete cause stays available as details.
/// It never describes the execution state of the retained world.
fn admission_refusal(edit: &Edit, error: &str) -> crate::notice::Notice {
    fn file_name(path: &std::path::Path) -> String {
        path.file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| path.display().to_string())
    }
    let primary = match edit {
        Edit::New => "Could not open the default scene.".into(),
        Edit::Scene(path) => format!("Could not open scene {}.", file_name(path)),
        Edit::Import { build, .. } => {
            format!("Could not admit robot build {}.", file_name(build))
        }
        Edit::Place => "Could not place the robot.".into(),
        Edit::CancelImport => "Could not cancel the placement.".into(),
        Edit::Remove => "Could not remove the robot.".into(),
        Edit::Undo => "Could not restore the removed robot.".into(),
    };
    crate::notice::Notice::new(
        primary,
        "The current simulation is kept. Correct the reported cause and try again.",
        error,
    )
}
fn digest(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    format!("{:x}", Sha256::digest(bytes))
}
fn options(world: &World) -> crate::config::Options {
    let mut options = crate::config::Options {
        scene: world
            .document
            .definition
            .scene
            .path
            .clone()
            .unwrap_or_else(|| "Bundled default scene".into()),
        bundle: world
            .prepared
            .as_ref()
            .map(|p| p.bundle().root.clone())
            .unwrap_or_default(),
        simulation_run: None,
        probe: false,
        json: false,
        presentation: crate::config::Presentation::Desktop,
        scope: Some("local".into()),
        connect: None,
        supervisor_id: Some("local".into()),
        run_id: Some(format!("desktop-{}", world.serial)),
        bound: None,
        auto_run: false,
    };
    if let Some(launch) = &world.launch {
        options.scope = launch.scope.clone();
        options.connect = launch.connect.clone();
        options.supervisor_id = launch.supervisor_id.clone();
        options.run_id = Some(format!(
            "{}-{}",
            launch.run_id.as_deref().unwrap_or("desktop"),
            world.serial
        ));
        options.bound = launch.bound;
        options.simulation_run = launch.simulation_run.clone();
    }
    options
}
fn run(
    initial: Option<crate::config::Options>,
    mut requests: mpsc::Receiver<Request>,
    display: &Arc<Mutex<RunState>>,
    quit: crate::cancellation::Cancellation,
) -> Result<super::RunOutcome, String> {
    let mut invocation_pending = initial.is_some();
    let mut world = World::admit(match &initial {
        Some(o) => Document::scene(&o.scene)?,
        None => Document::default_scene()?,
    })?;
    if let Some(initial) = initial {
        let target = world
            .scene
            .model()
            .eligible_attachments()
            .into_iter()
            .find(|a| a.0 == "robot_mount")
            .ok_or("Developer scene has no robot_mount attachment")?;
        world.edit(Edit::Import {
            build: initial.bundle.clone(),
            target: target.0,
            position: target.1,
            quaternion: target.2,
        })?;
        world.edit(Edit::Place)?;
        world.launch = Some(initial.into());
    }
    // Explicit robot execution owns native publications until Stop. Publish the
    // admitted document identity first so its header never depends on that exit.
    display
        .lock()
        .map_err(|_| "World state lock poisoned")?
        .document = world.document_view();
    eprintln!(
        "phoxal-simulator: native world admitted model={} quantum={}s",
        world.scene.model().identity().to_hex(),
        world.scene.quantum().as_seconds()
    );
    let mut explicit_outcome = None;
    let mut active = false;
    let mut running = false;
    let mut interaction = crate::execution::interaction::Interaction::default();
    let clock = Instant::now();
    let mut timing = crate::execution::timing::Timing::new(crate::execution::Pacing::Realtime);
    let mut pacing = crate::execution::Pacing::Realtime;
    let mut commands = None;
    let mut cancel = crate::cancellation::Cancellation::default();
    let mut last_frame = Instant::now() - Duration::from_secs(1);
    loop {
        if quit.is_cancelled() {
            if invocation_pending {
                explicit_outcome = Some(Ok(super::RunOutcome::Cancelled));
            }
            break;
        }
        if active && cancel.is_cancelled() {
            interaction.cancel(&mut world.scene, display)?;
            timing.pause(clock.elapsed());
            active = false;
            running = false;
            display.lock().unwrap().completion = Some(Completion::Stopped);
        }
        for _ in 0..8 {
            if quit.is_cancelled() {
                break;
            }
            match requests.try_recv() {
                Ok(Request::Shutdown) => break,
                Ok(Request::Edit {
                    edit,
                    cancel: admission_cancel,
                }) => {
                    let replaces = matches!(
                        edit,
                        Edit::New | Edit::Scene(_) | Edit::Place | Edit::Remove | Edit::Undo
                    );
                    if active || display.lock().unwrap().cleanup_failed {
                        display.lock().unwrap().document_error =
                            Some("Confirm Stop and cleanup before editing".into());
                        continue;
                    }
                    let outcome = world.edit_checked(edit.clone(), &admission_cancel);
                    let mut state = display.lock().unwrap();
                    match outcome {
                        Err(e) if !admission_cancel.is_cancelled() => {
                            state.document_error = Some(admission_refusal(&edit, &e));
                        }
                        Err(_) => {}
                        Ok(()) => {
                            state.document_error = None;
                            if replaces {
                                timing = crate::execution::timing::Timing::new(pacing);
                                state.completion = None;
                                state.error = None;
                                state.speed = None;
                                state.wall_seconds = 0.;
                            }
                        }
                    }
                    drop(state);
                    last_frame = Instant::now() - Duration::from_secs(1);
                }
                Ok(Request::Start {
                    command,
                    commands: receiver,
                    cancel: requested_cancel,
                }) => {
                    if active || display.lock().unwrap().cleanup_failed {
                        continue;
                    }
                    if let Err(e) = world.verify() {
                        if invocation_pending {
                            invocation_pending = false;
                            explicit_outcome = Some(Err(e.clone()));
                        }
                        display.lock().unwrap().error = Some(e.into());
                        display.lock().unwrap().document.busy = false;
                        display.lock().unwrap().finished = true;
                        continue;
                    }
                    if requested_cancel.is_cancelled() {
                        let mut s = display.lock().unwrap();
                        s.finished = true;
                        s.ready = true;
                        s.document.busy = false;
                        s.stopping = false;
                        s.completion = Some(Completion::Cancelled);
                        if invocation_pending {
                            invocation_pending = false;
                            explicit_outcome = Some(Ok(super::RunOutcome::Cancelled));
                        }
                        continue;
                    }
                    if matches!(command, Command::Reset) {
                        timing = crate::execution::timing::Timing::new(pacing);
                        world.scene.reset().map_err(|e| e.to_string())?;
                        world.frozen = None;
                        world.generation += 1;
                        world.view.reset(world.generation, display)?;
                        last_frame = Instant::now() - Duration::from_secs(1);
                        display.lock().unwrap().finished = true;
                        continue;
                    }
                    cancel = requested_cancel;
                    {
                        let mut s = display.lock().unwrap();
                        s.finished = false;
                        s.completion = None;
                        s.error = None;
                        s.document.busy = false;
                    }
                    if let Some(prepared) = &world.prepared {
                        world.serial += 1;
                        let outcome = crate::lifecycle::run_prepared(
                            options(&world),
                            Some(Worker {
                                commands: receiver,
                                display: display.clone(),
                                cancel: cancel.clone(),
                            }),
                            prepared,
                        );
                        let mut s = display.lock().unwrap();
                        s.running = false;
                        s.finished = true;
                        s.ready = true;
                        s.stopping = false;
                        s.dragging = false;
                        let terminal = s
                            .requests
                            .drag_mailbox
                            .begin
                            .as_ref()
                            .map(|b| b.id)
                            .unwrap_or(0);
                        s.requests.drag_mailbox.end(terminal);
                        s.phase = super::RunStatus::Idle;
                        s.completion = Some(
                            outcome
                                .as_ref()
                                .map(super::RunOutcome::completion)
                                .unwrap_or(Completion::Failed),
                        );
                        if let Err(e) = &outcome {
                            s.error = Some(e.clone().into());
                        }
                        if invocation_pending {
                            invocation_pending = false;
                            explicit_outcome = Some(outcome);
                        }
                        world.frozen = s
                            .native_state
                            .as_deref()
                            .cloned()
                            .filter(|s| s.model_identity() == world.scene.model().identity());
                        world.generation = s.generation;
                        drop(s);
                        world.view = Viewport::new(
                            world.scene.model(),
                            &format!("world-{}", world.serial),
                            world.generation,
                        )?;
                        last_frame = Instant::now() - Duration::from_secs(1);
                    } else {
                        timing = crate::execution::timing::Timing::new(pacing);
                        active = true;
                        commands = Some(receiver);
                    }
                }
                Err(mpsc::error::TryRecvError::Empty) => break,
                Err(mpsc::error::TryRecvError::Disconnected) => break,
            }
        }
        if active {
            if interaction.input(&mut world.scene, &mut world.view, display, !running)? {
                last_frame = Instant::now() - Duration::from_secs(1);
            }
            for _ in 0..8 {
                if cancel.is_cancelled() {
                    break;
                }
                match commands.as_mut().unwrap().try_recv() {
                    Ok(Command::Run) => {
                        timing.resume(
                            clock.elapsed(),
                            Duration::from_secs_f64(
                                world
                                    .scene
                                    .snapshot()
                                    .map_err(|e| e.to_string())?
                                    .time_seconds(),
                            ),
                        );
                        running = true;
                    }
                    Ok(Command::Pause) => {
                        timing.pause(clock.elapsed());
                        interaction.cancel(&mut world.scene, display)?;
                        running = false;
                    }
                    Ok(Command::Step) => {
                        running = false;
                        timing.resume(
                            clock.elapsed(),
                            Duration::from_secs_f64(
                                world
                                    .scene
                                    .snapshot()
                                    .map_err(|e| e.to_string())?
                                    .time_seconds(),
                            ),
                        );
                        world.scene.step().map_err(|e| e.to_string())?;
                        timing.pause(clock.elapsed());
                        break;
                    }
                    Ok(Command::Reset) => {
                        timing = crate::execution::timing::Timing::new(pacing);
                        interaction.cancel(&mut world.scene, display)?;
                        running = false;
                        world.scene.reset().map_err(|e| e.to_string())?;
                        world.generation += 1;
                        world.view.reset(world.generation, display)?;
                    }
                    Ok(Command::SetPacing(mode)) => {
                        pacing = mode;
                        timing.set_mode(
                            mode,
                            clock.elapsed(),
                            Duration::from_secs_f64(
                                world
                                    .scene
                                    .snapshot()
                                    .map_err(|e| e.to_string())?
                                    .time_seconds(),
                            ),
                        );
                    }
                    Ok(Command::Stop) => {
                        cancel.cancel();
                        break;
                    }
                    Err(_) => break,
                }
            }
            if running
                && !cancel.is_cancelled()
                && (pacing == crate::execution::Pacing::Fast
                    || timing
                        .delay(
                            clock.elapsed(),
                            Duration::from_secs_f64(
                                world
                                    .scene
                                    .snapshot()
                                    .map_err(|e| e.to_string())?
                                    .time_seconds(),
                            ),
                        )
                        .is_zero())
            {
                world.scene.step().map_err(|e| e.to_string())?;
            }
        }
        if active && cancel.is_cancelled() {
            interaction.cancel(&mut world.scene, display)?;
            timing.pause(clock.elapsed());
            active = false;
            running = false;
            display.lock().unwrap().completion = Some(Completion::Stopped);
            last_frame = Instant::now() - Duration::from_secs(1);
        }
        if last_frame.elapsed() >= Duration::from_millis(33) {
            if world.prepared.is_none() {
                let sim = Duration::from_secs_f64(
                    world
                        .scene
                        .snapshot()
                        .map_err(|e| e.to_string())?
                        .time_seconds(),
                );
                let mut state = display.lock().unwrap();
                state.wall_seconds = timing.wall(clock.elapsed()).as_secs_f64();
                state.speed = timing.speed(clock.elapsed(), sim);
            }
            world.publish(display, running, active, pacing)?;
            last_frame = Instant::now();
        }
        let delay = if running && pacing == crate::execution::Pacing::Fast {
            Duration::ZERO
        } else if running {
            timing.delay(
                clock.elapsed(),
                Duration::from_secs_f64(
                    world
                        .scene
                        .snapshot()
                        .map_err(|e| e.to_string())?
                        .time_seconds(),
                ),
            )
        } else {
            Duration::from_millis(1)
        };
        // One boundary per loop keeps the bounded command drain, Stop latch and
        // presentation live while catching up. Work/wake lateness stays owed to
        // the active-wall anchor; only an ahead-of-wall scene needs sleep.
        if delay.is_zero() {
            std::thread::yield_now();
        } else {
            std::thread::sleep(delay.min(Duration::from_millis(1)));
        }
    }
    let mut state = display
        .lock()
        .map_err(|_| "World state lock poisoned at shutdown")?;
    state.finished = true;
    state.running = false;
    state.ready = false;
    state.speed = None;
    state.requests = Default::default();
    state.selection = None;
    state.dragging = false;
    if state.cleanup_failed {
        let primary = state
            .error
            .as_ref()
            .map(|e| e.details.clone())
            .unwrap_or_default();
        let uncertainty =
            format!("{primary}\nExecution cleanup remains unconfirmed at world shutdown");
        if let Some(original) = explicit_outcome {
            // This later window recovery result cannot replace the original
            // contract-bearing invocation. Keep the fence and report it separately.
            eprintln!("phoxal-simulator: {uncertainty}");
            return original;
        }
        return Err(uncertainty);
    }
    explicit_outcome.unwrap_or(Ok(super::RunOutcome::Idle))
}

#[cfg(all(test, native_qualification))]
mod tests {
    use super::*;
    fn rover() -> PathBuf {
        std::env::var_os("PHOXAL_QUALIFICATION_BUILD")
            .expect("actual prepared rover")
            .into()
    }
    fn copy_tree(from: &std::path::Path, to: &std::path::Path) {
        std::fs::create_dir_all(to).unwrap();
        for entry in std::fs::read_dir(from).unwrap() {
            let entry = entry.unwrap();
            let target = to.join(entry.file_name());
            let kind = entry.file_type().unwrap();
            assert!(
                !kind.is_symlink(),
                "qualification copy never follows source symlinks"
            );
            if kind.is_dir() {
                copy_tree(&entry.path(), &target)
            } else {
                std::fs::copy(entry.path(), target).unwrap();
            }
        }
    }
    // Read-only copied-child comparison for retained native startup failures.
    // This changes no launch options, readiness budget or OS protection.
    fn executable_provenance(label: &str, build: &std::path::Path) {
        use std::os::unix::fs::PermissionsExt as _;
        let facts = crate::bundle::BundleFacts::load(build).unwrap();
        let path = facts.root.join(&facts.admitted.supervisor.path);
        let bytes = std::fs::read(&path).unwrap();
        eprintln!(
            "QUALIFICATION_EXECUTABLE {label} path={} mode={:o} bytes={} sha256={:x} header={:?}",
            path.display(),
            path.metadata().unwrap().permissions().mode(),
            bytes.len(),
            <sha2::Sha256 as sha2::Digest>::digest(&bytes),
            &bytes[..bytes.len().min(8)],
        );
        #[cfg(target_os = "macos")]
        let signature = std::process::Command::new("codesign")
            .args(["--verify", "--strict"])
            .arg(&path)
            .output()
            .unwrap();
        #[cfg(target_os = "macos")]
        eprintln!(
            "QUALIFICATION_SIGNATURE {label} status={} stderr={}",
            signature.status,
            String::from_utf8_lossy(&signature.stderr)
        );
    }
    fn import(build: PathBuf, position: [f64; 3]) -> Edit {
        Edit::Import {
            build,
            target: "robot_mount".into(),
            position,
            quaternion: [1., 0., 0., 0.],
        }
    }
    #[track_caller]
    fn wait(handle: &Handle, condition: impl Fn(&RunState) -> bool) {
        let limit = Instant::now() + Duration::from_secs(30);
        loop {
            let s = handle.display.lock().unwrap();
            if condition(&s) {
                return;
            }
            if Instant::now() >= limit {
                let detail = format!(
                    "error={:?},ready={},finished={},busy={},boundary={},generation={}",
                    s.error, s.ready, s.finished, s.document.busy, s.boundary, s.generation
                );
                drop(s);
                panic!("world timeout: {detail}");
            }
            drop(s);
            std::thread::sleep(Duration::from_millis(5));
        }
    }
    #[test]
    fn canceled_prepared_import_and_scene_change_preserve_confirmed_native_document() {
        let mut world = World::admit(Document::default_scene().unwrap()).unwrap();
        let before = world.snapshot().unwrap();
        let definition = world.document.definition.clone();
        let cancel = crate::cancellation::Cancellation::default();
        cancel.cancel();
        assert!(
            world
                .edit_checked(import(rover(), [1., 2., 0.]), &cancel)
                .is_err()
        );
        assert!(world.edit_checked(Edit::New, &cancel).is_err());
        assert_eq!(world.snapshot().unwrap(), before);
        assert_eq!(world.document.definition, definition);
        assert!(world.candidate.is_none());
        world.edit(import(rover(), [1., 2., 0.])).unwrap();
        let candidate = world
            .candidate
            .as_ref()
            .unwrap()
            .prepared
            .model()
            .identity()
            .to_owned();
        assert!(
            world
                .edit_checked(import(rover(), [4., 5., 0.]), &cancel)
                .is_err()
        );
        assert_eq!(
            world
                .candidate
                .as_ref()
                .unwrap()
                .prepared
                .model()
                .identity(),
            candidate
        );
        assert_eq!(world.snapshot().unwrap(), before);
    }
    #[test]
    fn native_default_and_rover_candidate_cancel_commit_integrity_and_reset() {
        let dir = tempfile::tempdir().unwrap();
        let build = dir.path().join("build");
        copy_tree(&rover(), &build);
        let mut world = World::admit(Document::default_scene().unwrap()).unwrap();
        assert_eq!(world.scene.quantum().as_seconds(), 0.002);
        assert!(world.prepared.is_none());
        let original = world.snapshot().unwrap();
        let authored = world.document.definition.clone();
        world.edit(import(build.clone(), [1., 2., 0.])).unwrap();
        assert_eq!(world.snapshot().unwrap(), original);
        assert_eq!(world.document.definition, authored);
        let model = world.candidate.as_ref().unwrap().prepared.model();
        let workspace = crate::mujoco::Workspace::new(model).unwrap();
        let initial = workspace.snapshot().unwrap();
        assert!((initial.qpos()[0] - 1.).abs() < 1e-10);
        assert!((initial.qpos()[1] - 2.).abs() < 1e-10);
        assert!(
            !workspace.contact_distances().is_empty(),
            "actual rover must have measured ground contacts"
        );
        eprintln!(
            "actual rover ground contacts {:?}; authored qpos {:?}; model {}",
            workspace.contact_distances(),
            initial.qpos(),
            model.identity().to_hex()
        );
        assert!(
            workspace.contact_distances().iter().all(|d| *d >= -1e-7),
            "authored rover intersects default ground: {:?}",
            workspace.contact_distances()
        );
        world.edit(Edit::CancelImport).unwrap();
        assert_eq!(world.snapshot().unwrap(), original);
        assert_eq!(world.document.definition, authored);
        world.edit(import(build.clone(), [1., 2., 0.])).unwrap();
        let selected = world.candidate.as_ref().unwrap().prepared.bundle();
        let root = selected.model.as_ref().unwrap().resources[0].clone();
        let component = selected.components.values().next().unwrap();
        let relative = selected.component_sources.get(&component.instance).unwrap();
        let (definition, _) = crate::bundle::component_definition(&component.definition);
        let paths = [
            build.join("manifest.json"),
            build.join(root),
            build.join(relative).join(&definition.file),
        ];
        for path in paths {
            let bytes = std::fs::read(&path).unwrap();
            let mut changed = bytes.clone();
            changed.extend_from_slice(b"\n");
            std::fs::write(&path, changed).unwrap();
            assert!(
                world.edit(Edit::Place).is_err(),
                "changed source committed: {path:?}"
            );
            assert_eq!(world.snapshot().unwrap(), original);
            assert_eq!(world.document.definition, authored);
            std::fs::write(path, bytes).unwrap();
        }
        world.edit(Edit::Place).unwrap();
        let placement = world.snapshot().unwrap();
        let display = Arc::new(Mutex::new(RunState::default()));
        world.view.publish(&mut display.lock().unwrap());
        assert_eq!(
            display.lock().unwrap().selection.as_ref().map(|s| s.body),
            world.scene.model().robot_root(),
            "confirmed Place did not select the actual robot root"
        );
        assert!(world.prepared.is_some());
        world.scene.advance(10).unwrap();
        assert!(!world.snapshot().unwrap().qpos().eq(placement.qpos()));
        world.scene.reset().unwrap();
        assert_eq!(world.snapshot().unwrap(), placement);
        world.edit(Edit::Remove).unwrap();
        assert!(world.prepared.is_none());
        assert!(build.is_dir());
        world.edit(Edit::Undo).unwrap();
        assert_eq!(world.snapshot().unwrap(), placement);
        let missing = Document {
            definition: crate::document::Definition {
                scene: crate::document::SceneSource::from_closed(
                    None,
                    &crate::mujoco::ClosedModel::from_xml("<mujoco/>").unwrap(),
                ),
                robot: None,
            },
        };
        let mut custom = World::admit(missing).unwrap();
        let before = custom.snapshot().unwrap();
        assert!(custom.edit(import(build, [0.; 3])).is_err());
        assert_eq!(custom.snapshot().unwrap(), before);
    }
    #[test]
    fn removed_identity_publication_tracks_retained_preparation_and_replacement() {
        let display = Arc::new(Mutex::new(RunState::default()));
        let publish = |world: &mut World| {
            world
                .publish(&display, false, false, crate::execution::Pacing::Realtime)
                .unwrap();
            display.lock().unwrap().document.clone()
        };
        for selection in [None, Some(0), Some(1), Some(2)] {
            let mut world = World::admit(Document::default_scene().unwrap()).unwrap();
            assert!(publish(&mut world).removed_robot.is_none());
            world.edit(import(rover(), [1., 0., 0.])).unwrap();
            world.edit(Edit::Place).unwrap();
            let original = world.snapshot().unwrap();
            assert_eq!(original.qpos()[0], 1.);
            let before = publish(&mut world);
            assert_eq!(before.robot.as_deref(), Some("robot-rover"));
            assert!(before.removed_robot.is_none());
            if let Some(body) = selection {
                let root = world.scene.model().robot_root().unwrap();
                let body = match body {
                    0 => 0,
                    1 => root,
                    _ => {
                        display
                            .lock()
                            .unwrap()
                            .bodies
                            .iter()
                            .find(|b| b.parent == Some(root))
                            .unwrap()
                            .id
                    }
                };
                world.view.select_body(body, &original).unwrap();
            } else {
                world.view.reset(0, &display).unwrap();
            }
            world.scene.step().unwrap();
            assert_ne!(world.snapshot().unwrap().qpos(), original.qpos());
            world.edit(Edit::Remove).unwrap();
            let removed = publish(&mut world);
            let retained = &world.removed.as_ref().unwrap().0;
            assert_eq!(
                removed.removed_robot.as_deref(),
                Some(retained.bundle().robot_id.as_str())
            );
            assert!(removed.robot.is_none());
            assert!(display.lock().unwrap().selection.is_none());
            world.scene.reset().unwrap();
            assert_eq!(
                publish(&mut world).removed_robot.as_deref(),
                Some("robot-rover")
            );
            world.edit(Edit::Undo).unwrap();
            assert_eq!(world.snapshot().unwrap(), original);
            let restored = publish(&mut world);
            assert_eq!(restored.robot.as_deref(), Some("robot-rover"));
            assert!(restored.removed_robot.is_none());
            world.edit(Edit::Remove).unwrap();
            world.edit(import(rover(), [2., 0., 0.])).unwrap();
            assert_eq!(
                publish(&mut world).removed_robot.as_deref(),
                Some("robot-rover")
            );
            world.edit(Edit::Place).unwrap();
            let replaced = publish(&mut world);
            assert_eq!(replaced.robot.as_deref(), Some("robot-rover"));
            assert!(replaced.removed_robot.is_none());
            assert_eq!(world.snapshot().unwrap().qpos()[0], 2.);
            world.edit(Edit::Remove).unwrap();
            world.edit(Edit::New).unwrap();
            let new = publish(&mut world);
            assert!(new.robot.is_none() && new.removed_robot.is_none());
        }
    }
    #[test]
    fn session_removal_undo_and_close_departure_work_without_persistence() {
        let mut handle = Handle::start(None).unwrap();
        wait(&handle, |s| s.ready && s.finished);
        handle.edit(import(rover(), [0.; 3])).unwrap();
        wait(&handle, |s| {
            !s.document.busy && s.document.candidate.is_some()
        });
        handle.edit(Edit::Place).unwrap();
        wait(&handle, |s| !s.document.busy && s.document.robot.is_some());
        handle.edit(Edit::Remove).unwrap();
        wait(&handle, |s| !s.document.busy && s.document.robot.is_none());
        assert_eq!(
            handle
                .display
                .lock()
                .unwrap()
                .document
                .removed_robot
                .as_deref(),
            Some("robot-rover")
        );
        assert!(handle.display.lock().unwrap().selection.is_none());
        let generation = handle.display.lock().unwrap().generation;
        handle.send(Command::Reset).unwrap();
        wait(&handle, |s| s.generation > generation && s.finished);
        assert_eq!(
            handle
                .display
                .lock()
                .unwrap()
                .document
                .removed_robot
                .as_deref(),
            Some("robot-rover")
        );
        handle.edit(Edit::Undo).unwrap();
        wait(&handle, |s| !s.document.busy && s.document.robot.is_some());
        assert!(
            handle
                .display
                .lock()
                .unwrap()
                .document
                .removed_robot
                .is_none()
        );
        assert!(handle.display.lock().unwrap().selection.is_none());
        assert!(handle.display.lock().unwrap().robot_root.is_some());
        handle.edit(Edit::Remove).unwrap();
        wait(&handle, |s| !s.document.busy && s.document.robot.is_none());
        handle.edit(Edit::Undo).unwrap();
        wait(&handle, |s| !s.document.busy && s.document.robot.is_some());
        // Read-only sessions depart without any unsaved-changes decision.
        let mut departure = crate::document::Departure::default();
        departure.begin(crate::document::PendingAction::Close);
        assert_eq!(
            departure.take_ready(true, false),
            Some(crate::document::PendingAction::Close)
        );
        assert_eq!(departure.take_ready(true, true), None);
        handle.shutdown().unwrap();
    }
    #[test]
    fn explicit_developer_failure_with_confirmed_cleanup_is_returned_after_join() {
        let dir = tempfile::tempdir().unwrap();
        let build = dir.path().join("build");
        executable_provenance("original-control", &rover());
        copy_tree(&rover(), &build);
        executable_provenance("copied-before-script", &build);
        let bundle = crate::bundle::BundleFacts::load(&build).unwrap();
        let supervisor = build.join(&bundle.admitted.supervisor.path);
        std::fs::write(
            &supervisor,
            "#!/bin/sh\nprintf 'EXPLICIT_READINESS_FAILURE\n' >&2\nexit 0\n",
        )
        .unwrap();
        executable_provenance("copied-script", &build);
        let options = crate::config::Options {
            scene: build.join("scene/scene.xml"),
            bundle: build,
            simulation_run: None,
            probe: false,
            json: false,
            presentation: crate::config::Presentation::Desktop,
            scope: Some("qualification".into()),
            connect: None,
            supervisor_id: Some("qualification".into()),
            run_id: Some("explicit-failure".into()),
            bound: None,
            auto_run: true,
        };
        let mut handle = Handle::start(Some(options)).unwrap();
        wait(&handle, |s| s.finished && s.error.is_some());
        {
            let state = handle.display.lock().unwrap();
            eprintln!(
                "QUALIFICATION_SCRIPT_ENTRY {}",
                state.error.as_ref().unwrap().details
            );
            assert!(
                !state.cleanup_failed,
                "fixture must qualify failure with confirmed cleanup"
            );
            assert!(
                state
                    .error
                    .as_ref()
                    .unwrap()
                    .details
                    .contains("EXPLICIT_READINESS_FAILURE")
            );
        }
        handle.edit(Edit::Remove).unwrap();
        wait(&handle, |s| !s.document.busy && s.document.robot.is_none());
        handle.edit(import(rover(), [0.; 3])).unwrap();
        wait(&handle, |s| {
            !s.document.busy && s.document.candidate.is_some()
        });
        handle.edit(Edit::Place).unwrap();
        wait(&handle, |s| !s.document.busy && s.document.robot.is_some());
        handle.send(Command::Step).unwrap();
        wait(&handle, |s| s.ready && !s.finished && s.boundary == 1);
        handle.send(Command::Stop).unwrap();
        wait(&handle, |s| s.finished);
        {
            let mut state = handle.display.lock().unwrap();
            state.cleanup_failed = true;
            state.error = Some("LATER_RELEASE_UNCONFIRMED".into());
        }
        let result = handle.shutdown();
        assert!(
            result.is_err(),
            "explicit developer execution failure was replaced with successful Idle"
        );
        assert!(result.err().unwrap().contains("EXPLICIT_READINESS_FAILURE"));
        assert!(handle.display.lock().unwrap().cleanup_failed);
    }
    #[test]
    fn original_executed_invocation_retains_real_report_after_later_document_and_fence() {
        let build = rover();
        let options = crate::config::Options {
            scene: build.join("scene/scene.xml"),
            bundle: build,
            simulation_run: None,
            probe: false,
            json: false,
            presentation: crate::config::Presentation::Desktop,
            scope: Some("qualification".into()),
            connect: None,
            supervisor_id: Some("qualification".into()),
            run_id: Some("original-completed".into()),
            bound: Some(crate::config::Bound::Steps(1)),
            auto_run: true,
        };
        let mut handle = Handle::start(Some(options)).unwrap();
        wait(&handle, |s| s.ready && s.boundary == 1);
        handle.send(Command::Stop).unwrap();
        wait(&handle, |s| s.finished);
        handle.edit(Edit::Remove).unwrap();
        wait(&handle, |s| !s.document.busy && s.document.robot.is_none());
        {
            let mut state = handle.display.lock().unwrap();
            state.cleanup_failed = true;
            state.error = Some("TEST_OWNED_LATER_UNCERTAINTY".into());
        }
        let outcome = handle
            .shutdown()
            .expect("later recovery fence replaced original completed invocation");
        assert_eq!(outcome.completion(), Completion::Completed);
        assert!(
            outcome.terminal().is_some(),
            "lost real original terminal evidence"
        );
        assert!(handle.display.lock().unwrap().cleanup_failed);
        assert!(handle.send(Command::Run).is_err());
    }
    #[test]
    fn foreign_host_robot_import_is_refused_and_preserves_confirmed_world() {
        let dir = tempfile::tempdir().unwrap();
        let build = dir.path().join("build");
        copy_tree(&rover(), &build);
        let manifest_path = build.join("manifest.json");
        let mut manifest: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&manifest_path).unwrap()).unwrap();
        manifest["target"] = if phoxal::artifact::application::HOST_EXECUTION_TARGET
            == "aarch64-unknown-linux-gnu"
        {
            "aarch64-apple-darwin"
        } else {
            "aarch64-unknown-linux-gnu"
        }
        .into();
        let bytes = serde_json::to_vec(&manifest).unwrap();
        std::fs::write(manifest_path, &bytes).unwrap();
        let mut current = World::admit(Document::default_scene().unwrap()).unwrap();
        let prior = current.snapshot().unwrap();
        let definition = current.document.definition.clone();
        assert!(
            current.edit(import(build, [0.; 3])).is_err(),
            "import admitted a foreign-host prepared robot"
        );
        assert_eq!(current.snapshot().unwrap(), prior);
        assert_eq!(current.document.definition, definition);
    }
    #[test]
    fn replacement_execution_uses_current_preparation_and_not_old_scenario() {
        let dir = tempfile::tempdir().unwrap();
        let first = dir.path().join("first");
        let second = dir.path().join("second");
        executable_provenance("original-control", &rover());
        copy_tree(&rover(), &first);
        copy_tree(&rover(), &second);
        executable_provenance("copied-first", &first);
        executable_provenance("copied-second", &second);
        let mut world =
            World::admit(Document::scene(&first.join("scene/scene.xml")).unwrap()).unwrap();
        world.edit(import(first.clone(), [0.; 3])).unwrap();
        world.edit(Edit::Place).unwrap();
        let mut initial = options(&world);
        initial.bundle = first;
        initial.simulation_run = Some(dir.path().join("original-scenario.json"));
        initial.bound = Some(crate::config::Bound::Steps(2));
        initial.auto_run = true;
        world.launch = Some(initial.into());
        world.edit(Edit::Remove).unwrap();
        world.edit(import(second.clone(), [0.; 3])).unwrap();
        world.edit(Edit::Place).unwrap();
        let execution = options(&world);
        assert_eq!(
            execution.bundle,
            second.canonicalize().unwrap(),
            "report/launch still uses the removed build"
        );
        assert!(
            execution.simulation_run.is_none(),
            "obsolete developer scenario followed a different imported robot"
        );
        let outcome =
            crate::lifecycle::run_prepared(execution, None, world.prepared.as_ref().unwrap())
                .unwrap();
        let terminal = outcome.terminal().expect("actual executed report");
        let crate::runtime::TerminalEvidence::V0 { model_identity, .. } = terminal;
        assert_eq!(*model_identity, world.scene.model().identity().to_hex());
    }
    #[test]
    fn native_default_scene_realtime_and_fast_measure_active_wall_with_render_cadence() {
        let mut handle = Handle::start(None).unwrap();
        let observed = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            wait(&handle, |s| s.ready && s.finished);
            assert!(handle.display.lock().unwrap().document.robot.is_none());
            handle.send(Command::Run).unwrap();
            wait(&handle, |s| s.running);
            let mut realtime_ratio = 0.;
            for mode in [
                crate::execution::Pacing::Realtime,
                crate::execution::Pacing::Fast,
                crate::execution::Pacing::Realtime,
            ] {
                handle.send(Command::SetPacing(mode)).unwrap();
                wait(&handle, |s| s.pacing == mode);
                let sample = || {
                    let s = handle.display.lock().unwrap();
                    (
                        s.native_state.as_ref().unwrap().time_seconds(),
                        s.wall_seconds,
                        s.speed,
                        s.boundary,
                    )
                };
                let first = sample();
                let start = Instant::now();
                let mut publications = 0;
                let mut previous = first.3;
                while start.elapsed() < Duration::from_secs(3) {
                    std::thread::sleep(Duration::from_millis(5));
                    let current = sample();
                    if current.3 != previous {
                        publications += 1;
                        previous = current.3;
                    }
                }
                let elapsed = start.elapsed().as_secs_f64();
                let last = sample();
                let logical = last.0 - first.0;
                let active_wall = last.1 - first.1;
                let ratio = logical / active_wall;
                println!(
                    "SCENE_HOST mode={mode:?} quantum=0.002 logical_delta={logical:.6} active_wall_delta={active_wall:.6} monotonic_window={elapsed:.6} logical_per_active_wall={ratio:.6} displayed_speed={:?} publications={publications}",
                    last.2
                );
                assert!((active_wall - elapsed).abs() < 0.15);
                assert!(
                    publications > 30 && publications < 110,
                    "bounded production render cadence"
                );
                assert!(
                    (last.2.unwrap() / ratio - 1.).abs() < 0.15,
                    "displayed rate must reflect measured progress"
                );
                if mode == crate::execution::Pacing::Realtime {
                    realtime_ratio = ratio;
                } else {
                    assert!(ratio > 2., "Fast remains uncapped");
                }
            }
            handle
                .send(Command::SetPacing(crate::execution::Pacing::Realtime))
                .unwrap();
            wait(&handle, |s| s.pacing == crate::execution::Pacing::Realtime);
            let command_start = Instant::now();
            handle.send(Command::Pause).unwrap();
            wait(&handle, |s| !s.running);
            println!(
                "SCENE_CONTROL pause_latency_ms={}",
                command_start.elapsed().as_millis()
            );
            let frozen = {
                let s = handle.display.lock().unwrap();
                assert!(s.speed.is_none());
                (
                    s.native_state.as_ref().unwrap().clone(),
                    s.wall_seconds,
                    s.boundary,
                )
            };
            std::thread::sleep(Duration::from_millis(120));
            {
                let s = handle.display.lock().unwrap();
                assert_eq!(s.native_state.as_ref().unwrap().as_ref(), frozen.0.as_ref());
                assert_eq!(s.wall_seconds, frozen.1);
            }
            handle.send(Command::Step).unwrap();
            wait(&handle, |s| s.boundary == frozen.2 + 1 && !s.running);
            let command_start = Instant::now();
            handle.send(Command::Stop).unwrap();
            wait(&handle, |s| s.finished && !s.stopping);
            println!(
                "SCENE_CONTROL stop_latency_ms={}",
                command_start.elapsed().as_millis()
            );
            assert!(
                realtime_ratio > 0.85 && realtime_ratio < 1.15,
                "host realtime progress {realtime_ratio}"
            );
        }));
        let cleanup = handle.shutdown();
        assert!(cleanup.is_ok());
        if let Err(panic) = observed {
            std::panic::resume_unwind(panic);
        }
    }
    #[test]
    fn native_scene_only_controls_freeze_step_reset_stop_edit_and_restart_without_robot() {
        let mut handle = Handle::start(None).unwrap();
        wait(&handle, |s| s.ready && s.finished);
        assert!(handle.display.lock().unwrap().document.robot.is_none());
        let initial = handle
            .display
            .lock()
            .unwrap()
            .native_state
            .as_ref()
            .unwrap()
            .clone();
        handle.send(Command::Step).unwrap();
        wait(&handle, |s| {
            s.ready && s.boundary == 1 && !s.running && !s.finished
        });
        let paused = handle
            .display
            .lock()
            .unwrap()
            .native_state
            .as_ref()
            .unwrap()
            .clone();
        std::thread::sleep(Duration::from_millis(120));
        assert_eq!(
            *handle
                .display
                .lock()
                .unwrap()
                .native_state
                .as_ref()
                .unwrap()
                .as_ref(),
            *paused
        );
        assert!(
            handle.edit(Edit::New).is_err(),
            "paused active execution cannot edit"
        );
        handle.send(Command::Run).unwrap();
        wait(&handle, |s| s.running && s.boundary > 1);
        handle.send(Command::Pause).unwrap();
        wait(&handle, |s| !s.running);
        handle.send(Command::Reset).unwrap();
        wait(&handle, |s| s.boundary == 0 && s.generation == 1);
        assert_eq!(
            handle
                .display
                .lock()
                .unwrap()
                .native_state
                .as_ref()
                .unwrap()
                .qpos(),
            initial.qpos()
        );
        handle.send(Command::Stop).unwrap();
        wait(&handle, |s| s.finished && !s.stopping);
        let stopped = handle
            .display
            .lock()
            .unwrap()
            .native_state
            .as_ref()
            .unwrap()
            .clone();
        std::thread::sleep(Duration::from_millis(80));
        assert_eq!(
            *handle
                .display
                .lock()
                .unwrap()
                .native_state
                .as_ref()
                .unwrap()
                .as_ref(),
            *stopped
        );
        handle.send(Command::Step).unwrap();
        wait(&handle, |s| s.ready && s.boundary == 1 && !s.finished);
        handle.send(Command::Stop).unwrap();
        wait(&handle, |s| s.finished);
        {
            let mut s = handle.display.lock().unwrap();
            s.cleanup_failed = true;
            s.error = Some("Authority release unconfirmed".into());
        }
        assert!(handle.edit(Edit::New).is_err());
        assert!(handle.send(Command::Run).is_err());
        {
            let s = handle.display.lock().unwrap();
            assert!(s.cleanup_failed);
            assert_eq!(
                s.error.as_ref().unwrap().primary,
                "Authority release unconfirmed"
            );
        }
        assert!(handle.edit(Edit::New).is_err());
        assert!(handle.send(Command::Run).is_err());
        assert!(handle.shutdown().is_err());
        assert!(handle.display.lock().unwrap().cleanup_failed);
    }
    #[test]
    fn native_product_import_deliberate_robot_step_cleanup_restart_and_source_change() {
        let mut handle = Handle::start(None).unwrap();
        wait(&handle, |s| s.ready && s.finished);
        handle.edit(import(rover(), [0.; 3])).unwrap();
        wait(&handle, |s| {
            !s.document.busy && s.document.candidate.is_some()
        });
        assert!(handle.display.lock().unwrap().finished);
        handle.edit(Edit::Place).unwrap();
        wait(&handle, |s| !s.document.busy && s.document.robot.is_some());
        let pose = handle
            .display
            .lock()
            .unwrap()
            .native_state
            .as_ref()
            .unwrap()
            .qpos()
            .to_vec();
        handle.send(Command::Step).unwrap();
        wait(&handle, |s| {
            s.ready && s.boundary == 1 && !s.running && !s.finished
        });
        handle.send(Command::Run).unwrap();
        wait(&handle, |s| s.ready && s.running && s.boundary >= 20);
        handle.send(Command::Pause).unwrap();
        wait(&handle, |s| s.ready && !s.running);
        let frozen = handle
            .display
            .lock()
            .unwrap()
            .native_state
            .as_ref()
            .unwrap()
            .clone();
        std::thread::sleep(Duration::from_millis(80));
        assert_eq!(
            handle
                .display
                .lock()
                .unwrap()
                .native_state
                .as_ref()
                .unwrap()
                .as_ref(),
            frozen.as_ref()
        );
        assert!(handle.edit(Edit::Remove).is_err());
        let generation = handle.display.lock().unwrap().generation;
        handle.send(Command::Reset).unwrap();
        wait(&handle, |s| {
            s.boundary == 0 && s.generation == generation + 1
        });
        assert_eq!(
            handle
                .display
                .lock()
                .unwrap()
                .native_state
                .as_ref()
                .unwrap()
                .qpos(),
            pose
        );
        handle.send(Command::Stop).unwrap();
        wait(&handle, |s| s.finished && !s.stopping);
        assert!(!handle.display.lock().unwrap().cleanup_failed);
        handle.send(Command::Step).unwrap();
        wait(&handle, |s| s.ready && s.boundary == 1 && !s.finished);
        handle.send(Command::Stop).unwrap();
        wait(&handle, |s| s.finished);
        handle.shutdown().unwrap();
    }
}

#[cfg(all(test, native_qualification))]
mod custom_tests {
    use super::*;
    #[test]
    fn custom_scene_native_physics_and_source_failure_preserve_confirmed_world() {
        let dir = tempfile::tempdir().unwrap();
        let scene = dir.path().join("scene.xml");
        std::fs::write(&scene,r#"<mujoco><option timestep="0.002"/><worldbody><geom type="plane" size="10 10 0.1"/><body name="ball" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body><site name="explicit" pos="1 0 0"/><body name="fixed" pos="1 0 0"><site name="nested" pos="0 0 0"/></body></worldbody></mujoco>"#).unwrap();
        let mut world = World::admit(Document::scene(&scene).unwrap()).unwrap();
        let initial = world.snapshot().unwrap();
        assert_eq!(world.scene.model().eligible_attachments()[0].0, "explicit");
        world.scene.advance(20).unwrap();
        let moving = world.snapshot().unwrap();
        assert!(moving.qpos()[2] < initial.qpos()[2]);
        world
            .edit(Edit::Import {
                build: std::env::var_os("PHOXAL_QUALIFICATION_BUILD")
                    .expect("actual rover")
                    .into(),
                target: "explicit".into(),
                position: [0.; 3],
                quaternion: [1., 0., 0., 0.],
            })
            .unwrap();
        assert_eq!(world.snapshot().unwrap(), moving);
        world.edit(Edit::CancelImport).unwrap();
        assert_eq!(world.snapshot().unwrap(), moving);
        let before = world.document.definition.clone();
        assert!(
            world
                .edit(Edit::Scene(dir.path().join("missing.xml")))
                .is_err()
        );
        assert_eq!(world.snapshot().unwrap(), moving);
        assert_eq!(world.document.definition, before);
        world.scene.reset().unwrap();
        assert_eq!(world.snapshot().unwrap(), initial);
        std::fs::write(scene, b"<mujoco><keyframe/></mujoco>").unwrap();
        assert!(world.verify().is_err());
        assert_eq!(world.snapshot().unwrap(), initial);
    }
}

#[cfg(all(test, native_qualification))]
mod developer_tests {
    use super::*;
    #[test]
    fn explicit_invocation_quit_before_first_start_returns_cancelled_without_report() {
        let build =
            PathBuf::from(std::env::var_os("PHOXAL_QUALIFICATION_BUILD").expect("actual rover"));
        let options = crate::config::Options {
            scene: build.join("scene/scene.xml"),
            bundle: build,
            simulation_run: None,
            probe: false,
            json: false,
            presentation: crate::config::Presentation::Desktop,
            scope: Some("qualification".into()),
            connect: None,
            supervisor_id: Some("qualification".into()),
            run_id: Some("early-cancel".into()),
            bound: None,
            auto_run: true,
        };
        let (_sender, receiver) = mpsc::channel(8);
        let display = Arc::new(Mutex::new(RunState::default()));
        let quit = crate::cancellation::Cancellation::default();
        quit.cancel();
        let outcome = run(Some(options), receiver, &display, quit).unwrap();
        assert!(
            matches!(outcome, super::super::RunOutcome::Cancelled),
            "original explicit invocation cancellation became idle"
        );
        assert!(
            outcome.terminal().is_none(),
            "cancellation before execution fabricated a report"
        );
        assert!(!display.lock().unwrap().cleanup_failed);
    }
    #[test]
    fn explicit_desktop_run_retains_paused_startup_and_auto_run_without_default_autostart() {
        let build =
            PathBuf::from(std::env::var_os("PHOXAL_QUALIFICATION_BUILD").expect("actual rover"));
        for auto_run in [false, true] {
            let options = crate::config::Options {
                scene: build.join("scene/scene.xml"),
                bundle: build.clone(),
                simulation_run: None,
                probe: false,
                json: false,
                presentation: crate::config::Presentation::Desktop,
                scope: Some("qualification".into()),
                connect: None,
                supervisor_id: Some("qualification".into()),
                run_id: Some("explicit-desktop".into()),
                bound: None,
                auto_run,
            };
            let mut handle = Handle::start(Some(options)).unwrap();
            let limit = Instant::now() + Duration::from_secs(30);
            loop {
                let s = handle.display.lock().unwrap();
                if s.ready && !s.finished && s.running == auto_run && (s.boundary > 0 || !auto_run)
                {
                    break;
                }
                let error = s.error.clone();
                drop(s);
                assert!(Instant::now() < limit, "explicit startup: {error:?}");
                std::thread::sleep(Duration::from_millis(5));
            }
            handle.send(Command::Stop).unwrap();
            handle.shutdown().unwrap();
            assert!(!handle.display.lock().unwrap().cleanup_failed);
        }
    }
}

#[cfg(all(test, native_qualification))]
mod stop_tests {
    use super::*;
    #[test]
    fn already_latched_stop_prevents_queued_stopped_reset_from_mutating_world() {
        let mut handle = Handle::start(None).unwrap();
        let limit = Instant::now() + Duration::from_secs(10);
        while !handle.display.lock().unwrap().ready {
            assert!(Instant::now() < limit);
            std::thread::sleep(Duration::from_millis(5));
        }
        let before = handle
            .display
            .lock()
            .unwrap()
            .native_state
            .as_ref()
            .unwrap()
            .clone();
        let (sender, receiver) = mpsc::channel(8);
        sender.try_send(Command::Reset).unwrap();
        let cancel = crate::cancellation::Cancellation::default();
        cancel.cancel();
        handle
            .requests
            .try_send(Request::Start {
                command: Command::Reset,
                commands: receiver,
                cancel,
            })
            .unwrap();
        loop {
            let s = handle.display.lock().unwrap();
            if s.completion == Some(Completion::Cancelled) {
                assert_eq!(s.generation, 0);
                assert_eq!(s.native_state.as_ref().unwrap().as_ref(), before.as_ref());
                break;
            }
            drop(s);
            assert!(Instant::now() < limit);
            std::thread::sleep(Duration::from_millis(5));
        }
        handle.shutdown().unwrap();
    }
}

#[cfg(all(test, native_qualification))]
mod epoch_tests {
    use super::*;
    #[test]
    fn same_model_new_document_epoch_accepts_presented_camera_without_dirtying() {
        let mut world = World::admit(Document::default_scene().unwrap()).unwrap();
        let display = Arc::new(Mutex::new(RunState::default()));
        world
            .publish(&display, false, false, crate::execution::Pacing::Realtime)
            .unwrap();
        let mut observed = display.lock().unwrap().observe();
        observed.requests.presented = Some(observed.frame.take().unwrap().view);
        display
            .lock()
            .unwrap()
            .request(observed.generation, observed.requests);
        world.edit(Edit::New).unwrap();
        world
            .publish(&display, false, false, crate::execution::Pacing::Realtime)
            .unwrap();
        let mut observed = display.lock().unwrap().observe();
        let view = observed.frame.take().unwrap().view;
        let mut camera = view.camera;
        camera.azimuth += 25.;
        observed.requests.presented = Some(view.clone());
        observed.requests.pending_camera = Some(crate::app::scene::CameraRequest {
            epoch: view.identity.epoch.clone(),
            camera,
        });
        display
            .lock()
            .unwrap()
            .request(observed.generation, observed.requests);
        world
            .publish(&display, false, false, crate::execution::Pacing::Realtime)
            .unwrap();
        assert_eq!(display.lock().unwrap().camera, Some(camera));
    }
}

#[cfg(all(test, native_qualification))]
mod render_failure_tests {
    use super::*;
    #[test]
    fn explicit_scene_render_admission_failure_preserves_previous_document_and_native_world() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("invalid-framebuffer.xml");
        std::fs::write(&file,r#"<mujoco><visual><global offwidth="100000" offheight="100000"/></visual><worldbody><geom type="plane" size="10 10 0.1"/></worldbody></mujoco>"#).unwrap();
        let mut world = World::admit(Document::default_scene().unwrap()).unwrap();
        let before = world.snapshot().unwrap();
        let definition = world.document.definition.clone();
        assert!(
            world.edit(Edit::Scene(file)).is_err(),
            "render-invalid scene replaced the confirmed world"
        );
        assert_eq!(world.snapshot().unwrap(), before);
        assert_eq!(world.document.definition, definition);
    }
}

#[cfg(all(test, native_qualification))]
mod close_lifetime_tests {
    use super::*;
    #[test]
    fn termination_after_robot_run_and_new_reaches_application_owner() {
        const CHILD_ENV: &str = "PHOXAL_CLOSE_LIFETIME_CHILD";
        let name = "app::world::close_lifetime_tests::termination_after_robot_run_and_new_reaches_application_owner";
        if let Some(marker) = std::env::var_os(CHILD_ENV) {
            let mut handle = Handle::start(None).unwrap();
            let wait = |condition: &dyn Fn(&RunState) -> bool| {
                let deadline = Instant::now() + Duration::from_secs(30);
                loop {
                    if condition(&handle.display.lock().unwrap()) {
                        break;
                    }
                    assert!(
                        Instant::now() < deadline,
                        "owned world did not reach requested boundary"
                    );
                    std::thread::sleep(Duration::from_millis(5));
                }
            };
            wait(&|s| s.ready && s.finished);
            handle
                .edit(Edit::Import {
                    build: std::env::var_os("PHOXAL_QUALIFICATION_BUILD")
                        .unwrap()
                        .into(),
                    target: "robot_mount".into(),
                    position: [1., 0., 0.],
                    quaternion: [1., 0., 0., 0.],
                })
                .unwrap();
            wait(&|s| !s.document.busy && s.document.candidate.is_some());
            handle.edit(Edit::Place).unwrap();
            wait(&|s| !s.document.busy && s.document.robot.is_some());
            handle.send(Command::Run).unwrap();
            wait(&|s| s.running && !s.finished);
            handle.send(Command::Pause).unwrap();
            wait(&|s| !s.running && !s.finished);
            handle.send(Command::Stop).unwrap();
            wait(&|s| s.finished && !s.stopping);
            handle.edit(Edit::Remove).unwrap();
            wait(&|s| !s.document.busy && s.document.robot.is_none());
            handle.edit(Edit::Undo).unwrap();
            wait(&|s| !s.document.busy && s.document.robot.is_some());
            handle.edit(Edit::New).unwrap();
            wait(&|s| !s.document.busy && s.document.robot.is_none() && s.ready && s.finished);
            std::fs::write(marker, b"new-ready-after-run").unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while !handle.display.lock().unwrap().close_requested && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(5));
            }
            let requested = handle.display.lock().unwrap().close_requested;
            eprintln!("CLOSE_LIFETIME_SIGNAL delivered-at-new-ready close_requested={requested}");
            let outcome = handle.shutdown();
            assert!(outcome.is_ok(), "owned join error: {:?}", outcome.err());
            assert!(
                requested,
                "SIGTERM was swallowed after completed robot execution"
            );
            return;
        }
        let dir = tempfile::tempdir().unwrap();
        let marker = dir.path().join("ready");
        let log = dir.path().join("child.log");
        let file = std::fs::File::create(&log).unwrap();
        let mut child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([name, "--exact", "--test-threads=1", "--nocapture"])
            .env(CHILD_ENV, &marker)
            .stdout(file.try_clone().unwrap())
            .stderr(file)
            .spawn()
            .unwrap();
        eprintln!(
            "CLOSE_LIFETIME_CHILD pid={} executable={}",
            child.id(),
            std::env::current_exe().unwrap().display()
        );
        let deadline = Instant::now() + Duration::from_secs(30);
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            while !marker.is_file() {
                if let Some(status) = child.try_wait().unwrap() {
                    return Err(format!("child exited before New/Ready: {status}"));
                }
                if Instant::now() >= deadline {
                    return Err("child did not enter New/Ready boundary".into());
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            assert_eq!(unsafe { libc::kill(child.id() as i32, libc::SIGTERM) }, 0);
            loop {
                if let Some(status) = child.try_wait().unwrap() {
                    return Ok(status);
                }
                if Instant::now() >= deadline {
                    return Err("child did not complete owned signal/cleanup boundary".into());
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        }));
        // Unconditional owned cleanup, with forced cleanup explicitly a failure.
        let forced = child.try_wait().unwrap().is_none();
        if forced {
            child.kill().unwrap();
        }
        child.wait().unwrap();
        eprintln!("{}", std::fs::read_to_string(log).unwrap());
        assert!(!forced, "forced subprocess cleanup is not success");
        let outcome = outcome.expect("subprocess observation panicked after unconditional cleanup");
        assert!(
            outcome.as_ref().is_ok_and(|s| s.success()),
            "signal/lifetime outcome: {outcome:?}"
        );
    }
}

/// Artifact smoke uses the same application-owned default world, without a GUI
/// or prepared robot. Each Step is an ordinary native boundary and render.
pub(crate) fn smoke() -> Result<(), String> {
    let mut handle = Handle::start(None)?;
    let result = (|| {
        let deadline = Instant::now() + Duration::from_secs(30);
        let wait = |boundary: u64| -> Result<(), String> {
            loop {
                let state = handle
                    .display
                    .lock()
                    .map_err(|_| "World state lock poisoned")?;
                if let Some(error) = &state.error {
                    return Err(error.details.clone());
                }
                if state.ready && state.boundary == boundary {
                    return Ok(());
                }
                drop(state);
                if Instant::now() >= deadline {
                    return Err(
                        "Default native smoke did not complete within readiness bound".into(),
                    );
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        };
        wait(0)?;
        for boundary in 1..=10 {
            handle.send(Command::Step)?;
            wait(boundary)?;
        }
        let state = handle
            .display
            .lock()
            .map_err(|_| "World state lock poisoned")?;
        let snapshot = state
            .native_state
            .as_ref()
            .ok_or("Native smoke has no snapshot")?;
        Ok(
            serde_json::json!({"schema":"phoxal/application-smoke/v0","scene":state.document.scene_name,"boundary":state.boundary,"time_seconds":snapshot.time_seconds(),"robot":state.document.robot,"model_identity":snapshot.model_identity().to_hex()}),
        )
    })();
    let cleanup = handle.shutdown();
    match (result, cleanup) {
        (Ok(report), Ok(_)) => {
            println!("{report}");
            Ok(())
        }
        (Err(error), Ok(_)) | (Ok(_), Err(error)) => Err(error),
        (Err(error), Err(cleanup)) => Err(format!("{error}; cleanup: {cleanup}")),
    }
}
