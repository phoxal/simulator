//! Desktop presentation. Only the worker owns simulation state and authority.

pub(super) mod scene;
mod theme;
pub(super) mod viewport;

#[cfg(test)]
mod tests;

use crate::config::Options;
use crate::execution::Pacing;
use crate::mujoco::{BodyMobility, NativeBody, NativeSelection, ViewCamera};
use crate::notice::Notice;
use eframe::egui;
use scene::{CameraRequest, PresentedView, SceneRequest, SceneResult, ViewportFrame};
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc::{self, Receiver, Sender};

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
pub(super) struct DisplayState {
    pub(super) robot_name: String,
    pub(super) boundary: u64,
    pub(super) time_seconds: f64,
    pub(super) wall_seconds: f64,
    pub(super) speed: Option<f64>,
    pub(super) pacing: Pacing,
    pub(super) generation: u64,
    pub(super) running: bool,
    pub(super) ready: bool,
    pub(super) finished: bool,
    pub(super) close_requested: bool,
    pub(super) error: Option<Notice>,
    pub(super) phase: &'static str,
    pub(super) stopping: bool,
    pub(super) cleanup_failed: bool,
    pub(super) frame: Option<ViewportFrame>,
    pub(super) camera: Option<ViewCamera>,
    pub(super) pending_camera: Option<CameraRequest>,
    pub(super) presented: Option<Arc<PresentedView>>,
    pub(super) pending_scene: Option<SceneRequest>,
    pub(super) scene_result: Option<SceneResult>,
    pub(super) drag_mailbox: scene::DragMailbox,
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

impl DisplayState {
    fn presentation(&mut self) -> Self {
        Self {
            robot_name: self.robot_name.clone(),
            boundary: self.boundary,
            time_seconds: self.time_seconds,
            wall_seconds: self.wall_seconds,
            speed: self.speed,
            pacing: self.pacing,
            generation: self.generation,
            running: self.running,
            ready: self.ready,
            finished: self.finished,
            close_requested: self.close_requested,
            error: self.error.clone(),
            phase: self.phase,
            stopping: self.stopping,
            cleanup_failed: self.cleanup_failed,
            frame: self.frame.take(),
            camera: self.camera,
            presented: self.presented.clone(),
            scene_result: self.scene_result.take(),
            drag_mailbox: scene::DragMailbox {
                ended: self.drag_mailbox.ended,
                ..Default::default()
            },
            dragging: self.dragging,
            interaction_error: self.interaction_error.clone(),
            bodies: self.bodies.clone(),
            selection: self.selection.clone(),
            controls: self.controls.clone(),
            actuation_boundary: self.actuation_boundary,
            actuation_products: self.actuation_products,
            ..Default::default()
        }
    }

    fn publish_ui(&mut self, mut ui: Self) {
        // UI cannot overwrite worker lifecycle, samples or authority.
        // Requests remain tied to the snapshot actually presented this frame.
        if !self.ready || self.finished || self.generation != ui.generation {
            return;
        }
        let Some(presented) = ui.presented.take() else {
            return;
        };
        if self
            .frame
            .as_ref()
            .is_some_and(|frame| frame.view.identity.epoch != presented.identity.epoch)
            || self
                .presented
                .as_ref()
                .is_some_and(|view| view.identity.epoch != presented.identity.epoch)
        {
            return;
        }
        self.presented = Some(presented);
        if ui.pending_camera.is_some() {
            self.pending_camera = ui.pending_camera;
        }
        if ui.pending_scene.is_some() {
            self.pending_scene = ui.pending_scene;
        }
        self.drag_mailbox.end(ui.drag_mailbox.ended);
        if let Some(begin) = ui.drag_mailbox.begin
            && begin.id > self.drag_mailbox.ended
        {
            self.drag_mailbox.begin = Some(begin);
        }
        if let Some(update) = ui.drag_mailbox.update
            && update.id > self.drag_mailbox.ended
        {
            self.drag_mailbox.update = Some(update);
        }
    }
}

pub(super) struct Worker {
    pub(super) commands: Receiver<Command>,
    pub(super) display: Arc<Mutex<DisplayState>>,
    pub(super) cancel: crate::cancellation::Cancellation,
}

struct Control {
    commands: Sender<Command>,
    cancel: crate::cancellation::Cancellation,
    thread:
        Option<std::thread::JoinHandle<Result<Option<crate::runtime::TerminalEvidence>, String>>>,
}

// The idle check owns no execution, process or preparation state. Its late result
// cannot change an opened simulation's lifecycle or cleanup fence.
type AvailabilityResult = Arc<Mutex<Option<Result<(), Notice>>>>;

struct AvailabilityCheck {
    cancel: crate::cancellation::Cancellation,
    result: AvailabilityResult,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl AvailabilityCheck {
    fn start() -> Result<Self, String> {
        Self::spawn(crate::native_binding::initialize_with_cancel)
    }

    fn spawn(
        check: impl FnOnce(&crate::cancellation::Cancellation) -> Result<(), Notice> + Send + 'static,
    ) -> Result<Self, String> {
        let cancel = crate::cancellation::Cancellation::default();
        let checking = cancel.clone();
        let result = Arc::new(Mutex::new(None));
        let completed = result.clone();
        let thread = std::thread::Builder::new()
            .name("native-availability".into())
            .spawn(move || {
                let outcome = check(&checking);
                if checking.is_cancelled() {
                    return;
                }
                if let Err(error) = &outcome {
                    eprintln!("phoxal-simulator: {error}");
                }
                if let Ok(mut result) = completed.lock() {
                    *result = Some(outcome);
                }
            })
            .map_err(|error| format!("Cannot start native availability check: {error}"))?;
        Ok(Self {
            cancel,
            result,
            thread: Some(thread),
        })
    }

    fn finish(mut self) -> Result<(), String> {
        self.cancel.cancel();
        self.thread
            .take()
            .expect("availability check owns its thread")
            .join()
            .map_err(|_| "Native availability checker panicked during shutdown".into())
    }
}

impl Drop for AvailabilityCheck {
    fn drop(&mut self) {
        self.cancel.cancel();
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            eprintln!("Native availability checker panicked during shutdown");
        }
    }
}

fn start_worker(options: Options, display: Arc<Mutex<DisplayState>>) -> Result<Control, String> {
    *display.lock().map_err(|_| "desktop state lock poisoned")? = DisplayState::default();
    let (commands, receiver) = mpsc::channel(8);
    let cancel = crate::cancellation::Cancellation::default();
    let worker = Worker {
        commands: receiver,
        display: display.clone(),
        cancel: cancel.clone(),
    };
    let thread = std::thread::Builder::new()
        .name("simulation".into())
        .spawn(move || {
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                crate::lifecycle::run_owned(options, Some(worker))
            }))
            .unwrap_or_else(|_| {
                if let Ok(mut state) = display.lock() {
                    state.cleanup_failed = true;
                }
                Err("Simulation worker panicked; cleanup could not be acknowledged. Restart is disabled.".into())
            });
            // Finished means the worker returned; cleanup_failed separately fences restart.
            if let Ok(mut state) = display.lock() {
                state.running = false;
                state.finished = true;
                state.stopping = false;
                state.ready = false;
                state.controls.clear();
                state.selection = None;
                state.pending_scene = None;
                state.pending_camera = None;
                state.presented = None;
                state.scene_result = None;
                state.drag_mailbox = scene::DragMailbox::default();
                state.dragging = false;
                state.interaction_error = None;
                state.actuation_boundary = None;
                state.actuation_products = 0;
                if let Err(error) = &outcome {
                    if let Some(notice)=state.error.as_mut() { notice.details=error.clone(); } else { state.error = Some(error.clone().into()); }
                }
            }
            outcome
        })
        .map_err(|e| e.to_string())?;
    Ok(Control {
        commands,
        cancel,
        thread: Some(thread),
    })
}

pub(super) fn run(options: Options) -> Result<Option<crate::runtime::TerminalEvidence>, String> {
    open(Some(options))
}

pub(super) fn idle() -> Result<Option<crate::runtime::TerminalEvidence>, String> {
    open(None)
}

fn open(options: Option<Options>) -> Result<Option<crate::runtime::TerminalEvidence>, String> {
    let availability = options
        .is_none()
        .then(AvailabilityCheck::start)
        .transpose()?;
    let availability_result = availability.as_ref().map(|check| check.result.clone());
    let display = Arc::new(Mutex::new(DisplayState {
        finished: options.is_none(),
        ..Default::default()
    }));
    let control = match &options {
        Some(options) => start_worker(options.clone(), display.clone())?,
        None => {
            let (commands, _) = mpsc::channel(1);
            Control {
                commands,
                cancel: crate::cancellation::Cancellation::default(),
                thread: None,
            }
        }
    };
    let build_path = options
        .as_ref()
        .map(|options| options.bundle.display().to_string())
        .unwrap_or_default();
    let scene_path = options
        .as_ref()
        .map(|options| options.scene.display().to_string())
        .unwrap_or_default();
    let control = Arc::new(Mutex::new(control));
    let shutdown = control.clone();
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 820.0])
            .with_min_inner_size([480.0, 420.0]),
        ..Default::default()
    };
    let gui = eframe::run_native(
        "Phoxal Simulator",
        native_options,
        Box::new(move |cc| {
            theme::apply(&cc.egui_ctx);
            Ok(Box::new(Desktop {
                control,
                display,
                options,
                build_path,
                scene_path,
                restart: 0,
                texture: None,
                message: None,
                view_camera: None,
                gesture: None,
                next_gesture: 0,
                pointer_cut: None,
                closing: false,
                availability: availability_result,
                #[cfg(test)]
                last_viewport: None,
            }))
        }),
    )
    .map_err(|e| e.to_string());
    let mut owner = shutdown
        .lock()
        .map_err(|_| "desktop control lock poisoned")?;
    let _ = owner.commands.try_send(Command::Stop);
    owner.cancel.cancel();
    // Closing the command channel also stops the run if the finite queue is full.
    let (disconnected, _) = mpsc::channel(1);
    owner.commands = disconnected;
    let thread = owner.thread.take();
    drop(owner);
    let simulation = match thread {
        Some(thread) => thread.join().unwrap_or_else(|_| {
            Err("simulation worker panicked; cleanup is unconfirmed".to_owned())
        }),
        None => Ok(None),
    };
    let checker = availability
        .map(AvailabilityCheck::finish)
        .unwrap_or(Ok(()));
    let simulation = match (simulation, checker) {
        (outcome, Ok(())) => outcome,
        (Ok(_), Err(cleanup)) => Err(cleanup),
        (Err(primary), Err(cleanup)) => Err(format!(
            "{primary}\nAvailability checker shutdown: {cleanup}"
        )),
    };
    desktop_outcome(gui, simulation)
}

fn desktop_outcome<T>(gui: Result<(), String>, simulation: Result<T, String>) -> Result<T, String> {
    match (gui, simulation) {
        (Ok(()), outcome) => outcome,
        (Err(primary), Ok(_)) => Err(graphics_notice(primary, None).to_string()),
        (Err(primary), Err(cleanup)) => Err(graphics_notice(primary, Some(cleanup)).to_string()),
    }
}

fn graphics_notice(primary: String, cleanup: Option<String>) -> Notice {
    Notice::new(
        format!(
            "Desktop graphics/window creation failed: {}",
            primary.lines().next().unwrap_or("graphics unavailable")
        ),
        "run in a working graphical desktop session, or use `phoxal-simulator run --help` for explicit headless execution.",
        cleanup.map_or_else(
            || primary.clone(),
            |cleanup| format!("{primary}\nSimulation worker outcome: {cleanup}"),
        ),
    )
}

struct Desktop {
    availability: Option<AvailabilityResult>,
    control: Arc<Mutex<Control>>,
    display: Arc<Mutex<DisplayState>>,
    options: Option<Options>,
    build_path: String,
    scene_path: String,
    restart: u64,
    texture: Option<egui::TextureHandle>,
    message: Option<Notice>,
    view_camera: Option<ViewCamera>,
    gesture: Option<(u64, scene::SceneEpoch, egui::Pos2, f32)>,
    next_gesture: u64,
    pointer_cut: Option<viewport::PointerCut>,
    closing: bool,
    #[cfg(test)]
    last_viewport: Option<egui::Rect>,
}

impl Desktop {
    fn send(&mut self, command: Command) {
        if matches!(command, Command::Stop) {
            if let Ok(owner) = self.control.lock() {
                owner.cancel.cancel();
            }
            return;
        }
        self.message = self
            .control
            .lock()
            .map_err(|_| "desktop control lock poisoned".to_owned())
            .and_then(|owner| {
                owner
                    .commands
                    .try_send(command)
                    .map_err(|e| format!("Control unavailable: {e}"))
            })
            .err()
            .map(Notice::from);
    }
}

impl eframe::App for Desktop {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.draw(ui);
    }
}
impl Desktop {
    fn draw(&mut self, ui: &mut egui::Ui) {
        ui.ctx()
            .request_repaint_after(std::time::Duration::from_millis(33));
        let display = self.display.clone();
        let Ok(mut shared) = display.lock() else {
            ui.label("Simulation state is unavailable");
            return;
        };
        let mut state = shared.presentation();
        drop(shared);
        let availability_error = if self.options.is_none() {
            self.availability
                .as_ref()
                .and_then(|result| result.lock().ok()?.as_ref()?.as_ref().err().cloned())
        } else {
            None
        };
        let pin_notice = state.finished || state.error.is_some();
        if ui.ctx().input(|input| input.viewport().close_requested()) && !state.finished {
            self.closing = true;
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.send(Command::Stop);
            state.stopping = true;
        }
        if self.closing && state.finished {
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if state.close_requested && state.finished {
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if let Some(result) = state.scene_result.take()
            && state
                .presented
                .as_ref()
                .is_some_and(|view| view.identity.epoch == result.frame.epoch)
        {
            if result.stale {
                self.message = Some("View changed; select again on the current frame".into());
            } else {
                self.message = None;
                if let Some(camera) = result.camera {
                    self.view_camera = Some(camera);
                }
            }
        }
        if let Some(frame) = state.frame.take() {
            if self.view_camera.is_none()
                || state
                    .presented
                    .as_ref()
                    .is_none_or(|view| view.identity.epoch != frame.view.identity.epoch)
            {
                self.view_camera = Some(frame.view.camera);
            }
            state.presented = Some(frame.view);
            let image = egui::ColorImage::from_rgb(frame.image.resolution(), frame.image.rgb());
            if let Some(texture) = &mut self.texture {
                texture.set(image, egui::TextureOptions::LINEAR);
            } else {
                self.texture = Some(ui.ctx().load_texture(
                    "scene",
                    image,
                    egui::TextureOptions::LINEAR,
                ));
            }
        }
        egui::Frame::new()
            .fill(theme::SURFACE)
            .inner_margin(12)
            .show(ui, |ui| {
                ui.style_mut().spacing.item_spacing = egui::vec2(8.0, 8.0);
                ui.style_mut().spacing.button_padding = egui::vec2(10.0, 6.0);
                ui.horizontal_wrapped(|ui| {
                    ui.label(egui::RichText::new("Phoxal Simulator").size(18.0).strong());
                    if !state.robot_name.is_empty() { ui.label(&state.robot_name); }
                    ui.separator();
                    let status = if state.cleanup_failed && state.finished {
                        "Cleanup incomplete"
                    } else if state.stopping {
                        "Stopping"
                    } else if state.error.is_some() {
                        "Failed"
                    } else if !state.finished && !state.ready {
                        if state.phase.is_empty() { "Preparing" } else { state.phase }
                    } else if self.options.is_none() {
                        "Idle"
                    } else if state.finished && !state.cleanup_failed {
                        "Stopped"
                    } else if !state.ready {
                        if state.phase.is_empty() { "Preparing" } else { state.phase }
                    } else if state.running {
                        "Running"
                    } else {
                        "Paused"
                    };
                    ui.label(egui::RichText::new(status).color(if state.error.is_some() || state.cleanup_failed {
                        theme::ERROR
                    } else {
                        if state.stopping { theme::WARNING }
                        else if state.ready && state.running { theme::SUCCESS }
                        else { theme::MUTED }
                    }));
                });
                if pin_notice {
                    notice_primary(ui, &state, self.message.as_ref().or(availability_error.as_ref()));
                }
                if state.finished && !state.cleanup_failed {
                    let height = (ui.available_height() * 0.45).max(1.0);
                    egui::ScrollArea::vertical().id_salt("idle_inputs").max_height(height).show(ui, |ui| {
                    ui.add_space(16.0);
                    ui.label("Robot build directory");
                    ui.horizontal(|ui| {
                        path_input(ui, &mut self.build_path);
                        if ui.button("Browse…").clicked()
                            && let Some(path) = rfd::FileDialog::new().set_title("Choose runnable robot build directory").pick_folder() {
                            self.build_path = path.display().to_string();
                        }
                    });
                    ui.label("Scene file");
                    ui.horizontal(|ui| {
                        path_input(ui, &mut self.scene_path);
                        if ui.button("Browse…").clicked()
                            && let Some(path) = rfd::FileDialog::new().set_title("Choose simulation scene").add_filter("MuJoCo scene", &["xml", "mjz"]).pick_file() {
                            self.scene_path = path.display().to_string();
                        }
                    });
                    let selected = !self.build_path.trim().is_empty() && !self.scene_path.trim().is_empty();
                    if ui.add_enabled(selected, egui::Button::new("Open simulation")).clicked() {
                        let options = Options {
                            simulation_run: None, probe: false,
                            scene: self.scene_path.trim().into(), bundle: self.build_path.trim().into(),
                            json: false, presentation: crate::config::Presentation::Desktop,
                            scope: Some("local".into()), connect: None,
                            supervisor_id: Some("local".into()), run_id: Some("desktop".into()),
                            bound: None, auto_run: true,
                        };
                        let result = (|| -> Result<(), Notice> {
                            if !std::path::Path::new(&self.build_path).is_dir() { return Err(Notice::new("Robot build directory does not exist.", "choose a prepared runnable build directory.", format!("Selected build: {}",self.build_path))); }
                            if !std::path::Path::new(&self.scene_path).is_file() { return Err(Notice::new("Scene file does not exist.", "choose an MJCF or MJZ scene file.", format!("Selected scene: {}",self.scene_path))); }
                            let mut owner = self.control.lock().map_err(|_| "desktop control lock poisoned")?;
                            if let Some(thread) = owner.thread.take() {
                                // The previous execution has already joined all child processes.
                                let _ = thread.join().map_err(|_| "simulation worker panicked")?;
                            }
                            *owner = start_worker(options.clone(), self.display.clone())?;
                            self.options = Some(options);
                            state = DisplayState::default();
                            self.texture = None;
                            self.view_camera = None;
                            Ok(())
                        })();
                        self.message = result.err();
                    }
                    });
                }
                ui.add_space(12.0);
                ui.horizontal_wrapped(|ui| {
                    ui.add_enabled_ui(
                        state.ready && !state.finished && state.error.is_none(),
                        |ui| {
                            if ui
                                .button(if state.running { "Pause" } else { "Run" })
                                .clicked()
                            {
                                self.send(if state.running {
                                    Command::Pause
                                } else {
                                    Command::Run
                                });
                            }
                            if ui
                                .add_enabled(!state.running, egui::Button::new("Step"))
                                .on_disabled_hover_text("Available after startup; pause before stepping.")
                                .clicked()
                            {
                                self.send(Command::Step);
                            }
                            if ui
                                .add_enabled(!state.running, egui::Button::new("Reset"))
                                .on_disabled_hover_text("Available after startup; pause before resetting.")
                                .clicked()
                            {
                                self.send(Command::Reset);
                            }
                        },
                    );
                    if !state.finished
                        && ui.add_enabled(!state.stopping, egui::Button::new(if state.ready { "Stop" } else { "Cancel startup" })).clicked() {
                        state.stopping = true;
                        self.send(Command::Stop);
                    }
                    if state.finished && !state.cleanup_failed && self.options.is_some() && ui.button("Restart").clicked() {
                        let result = (|| -> Result<(), String> {
                            let mut owner = self
                                .control
                                .lock()
                                .map_err(|_| "desktop control lock poisoned")?;
                            if let Some(thread) = owner.thread.take() {
                                let _ = thread.join().map_err(|_| "simulation worker panicked")?;
                            }
                            self.restart += 1;
                            let mut options = self.options.clone().ok_or("no execution is selected")?;
                            options.run_id = Some(format!(
                                "{}-restart-{}",
                                options.run_id.as_deref().unwrap_or("desktop"),
                                self.restart
                            ));
                            options.auto_run = true;
                            state = DisplayState::default();
                            self.texture = None;
                            self.view_camera = None;
                            *owner = start_worker(options, self.display.clone())?;
                            Ok(())
                        })();
                        self.message = result.err().map(Notice::from);
                    }
                    ui.separator();
                    ui.add_enabled_ui(state.ready && !state.finished, |ui| {
                        for mode in [Pacing::Realtime, Pacing::Fast] {
                            if ui.selectable_label(state.pacing == mode, mode.label()).clicked() {
                                self.send(Command::SetPacing(mode));
                            }
                        }
                    });
                    ui.separator();
                    let speed = state.speed.map(|speed| format!("{speed:.2}x"))
                        .unwrap_or_else(|| "--".into());
                    ui.monospace(format!("Sim {:.2}s | Wall {:.2}s | Speed {speed}",
                        state.time_seconds, state.wall_seconds));
                });
                ui.add_space(6.0);
                ui.horizontal_wrapped(|ui| {
                    ui.add_enabled_ui(state.ready && !state.finished && state.presented.is_some(), |ui| {
                        if ui.add_enabled(state.selection.is_some(), egui::Button::new("Focus selected")).clicked() { queue_scene(&mut state, scene::SceneAction::Focus); }
                        if ui.button("Default view").clicked() { queue_scene(&mut state, scene::SceneAction::DefaultCamera); }
                    });
                    let hints = "Click: select · Primary drag: push / paused free-body move · Escape: cancel · Right drag: orbit · Shift-right / middle drag: pan · Scroll / pinch: zoom";
                    if ui.available_width() < 720.0 {
                        ui.weak("Click: select · Drag: interact · Scroll: zoom").on_hover_text(hints);
                    } else {
                        ui.weak(hints);
                    }
                });
                let narrow = ui.available_width() < 720.0;
                if narrow {
                    // Ancillary content shares one finite region, including
                    // expanded headers and every selected-body field.
                    let height = (ui.available_height() * 0.4).clamp(1.0, 220.0);
                    egui::ScrollArea::vertical()
                        .id_salt("narrow_scene_and_diagnostics")
                        .max_height(height)
                        .show(ui, |ui| {
                            diagnostics(ui, &state, self.message.as_ref().map(|notice|notice.details.as_str()), self.pointer_cut);
                            egui::CollapsingHeader::new("Scene and selected body")
                                .show(ui, |ui| { inspector(ui, &mut state, 140.0); });
                            if pin_notice { notice_details(ui, &state, self.message.as_ref().or(availability_error.as_ref())); }
                            else { notice(ui, &state, self.message.as_ref().or(availability_error.as_ref())); }
                        });
                    self.viewport(ui, &mut state);
                } else {
                    diagnostics(ui, &state, self.message.as_ref().map(|notice|notice.details.as_str()), self.pointer_cut);
                    ui.horizontal_top(|ui| {
                        let height = ui.available_height().max(1.0);
                        ui.allocate_ui_with_layout(egui::vec2(220.0,height), egui::Layout::top_down(egui::Align::Min), |ui| {
                            egui::ScrollArea::vertical().id_salt("wide_scene_inspector")
                                .max_height(height).show(ui, |ui| {
                                    if pin_notice { notice_details(ui, &state, self.message.as_ref().or(availability_error.as_ref())); }
                                    else { notice(ui, &state, self.message.as_ref().or(availability_error.as_ref())); }
                                    inspector(ui, &mut state, (height-180.0).max(60.0));
                                });
                        });
                        ui.separator();
                        self.viewport(ui, &mut state);
                    });
                }
            });
        let stopping = self
            .control
            .lock()
            .is_ok_and(|owner| owner.cancel.is_cancelled());
        if let Ok(mut shared) = display.lock() {
            if stopping && !shared.finished {
                shared.stopping = true;
            }
            shared.publish_ui(state);
        }
    }
}

impl Desktop {
    fn viewport(&mut self, ui: &mut egui::Ui, state: &mut DisplayState) {
        if state.dragging {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
            ui.label(if state.running {
                "Dragging with physical force"
            } else {
                "Repositioning free body"
            });
        }
        if let Some(error) = &state.interaction_error {
            ui.colored_label(theme::ERROR, error);
        }
        if let Some(texture) = &self.texture {
            let pickable = state.ready
                && !state.finished
                && state
                    .presented
                    .as_ref()
                    .is_some_and(|view| Some(view.camera) == self.view_camera);
            let camera = (state.ready && !state.finished)
                .then_some(self.view_camera)
                .flatten();
            let input = viewport::show(ui, texture, camera, pickable);
            if let Some(cut) = input.pointer_cut {
                self.pointer_cut = Some(cut);
            }
            #[cfg(test)]
            {
                self.last_viewport = input.canvas;
            }
            if let (Some(camera), Some(view)) = (input.camera, state.presented.as_ref()) {
                self.view_camera = Some(camera);
                state.pending_camera = Some(CameraRequest {
                    epoch: view.identity.epoch.clone(),
                    camera,
                });
            }
            if input.cancel || !state.ready || state.finished {
                if let Some((id, _, _, _)) = self.gesture.take() {
                    state.drag_mailbox.end(id);
                }
            } else {
                if let (Some((start, xy, height)), Some(view)) =
                    (input.begin, state.presented.as_ref())
                {
                    if let Some((id, _, _, _)) = self.gesture.take() {
                        state.drag_mailbox.end(id);
                    }
                    self.next_gesture = self.next_gesture.saturating_add(1);
                    let id = self.next_gesture;
                    self.gesture = Some((id, view.identity.epoch.clone(), start, height));
                    state.drag_mailbox.begin = Some(scene::DragBegin {
                        id,
                        frame: view.identity.clone(),
                        xy,
                        received: std::time::Instant::now(),
                    });
                }
                if let (Some((id, epoch, start, height)), Some((position, _))) =
                    (&self.gesture, input.held)
                {
                    state.drag_mailbox.update = Some(scene::DragUpdate {
                        id: *id,
                        epoch: epoch.clone(),
                        delta: viewport::drag_displacement(*start, position, *height),
                        received: std::time::Instant::now(),
                    });
                }
            }
            if let Some(xy) = input.pick {
                queue_scene(state, scene::SceneAction::Pick(xy));
            }
        } else {
            let available = ui.available_size().max(egui::vec2(1.0, 1.0));
            ui.allocate_ui_with_layout(
                available,
                egui::Layout::top_down(egui::Align::Center),
                |ui| {
                    ui.add_space((available.y / 2.0 - 30.0).max(0.0));
                    if state.error.is_some() {
                        ui.label("Native scene unavailable");
                    } else if state.finished {
                        ui.label("Open a robot build and scene to begin");
                    } else {
                        ui.spinner();
                        ui.label("Preparing native scene…");
                    }
                },
            );
        }
    }
}
fn notice(ui: &mut egui::Ui, state: &DisplayState, message: Option<&Notice>) {
    notice_primary(ui, state, message);
    notice_details(ui, state, message);
}
fn notice_primary(ui: &mut egui::Ui, state: &DisplayState, message: Option<&Notice>) {
    if state.cleanup_failed {
        ui.colored_label(theme::ERROR, "Cleanup incomplete: remote authority/session release or owned process/reader cleanup is unconfirmed. Restart is disabled.");
    }
    if let Some(error) = state.error.as_ref().or(message) {
        let first = &error.primary;
        ui.add(egui::Label::new(egui::RichText::new(first).color(theme::ERROR)).wrap());
        ui.label(if state.cleanup_failed {
            "Next: close this window, resolve the shutdown failure in Error details, and confirm the execution's participants have stopped and remote authority/session cleanup is complete before reopening the simulator.".to_owned()
        } else {
            // Producing boundaries supply their action. Projection keeps it intact,
            // without inferring remediation from phase names or native details.
            format!("Next: {}", error.action)
        });
    }
}
fn notice_details(ui: &mut egui::Ui, state: &DisplayState, message: Option<&Notice>) {
    if let Some(error) = state.error.as_ref().or(message) {
        ui.collapsing("Error details", |ui| {
            if ui.button("Copy details").clicked() {
                ui.ctx().copy_text(error.details.clone());
            }
            egui::ScrollArea::vertical()
                .max_height(120.0)
                .show(ui, |ui| {
                    ui.add(egui::Label::new(&error.details).selectable(true));
                });
        });
    }
    if state.finished
        && !state.cleanup_failed
        && state.error.as_ref().or(message).is_some_and(|error| {
            error
                .primary
                .starts_with("MuJoCo 3.12.0 unavailable or incompatible:")
        })
        && std::env::var_os("PHOXAL_MUJOCO_LIBRARY").is_none()
    {
        ui.horizontal_wrapped(|ui| {
            ui.label("Install explicitly in a terminal, then Open simulation or Restart:");
            if let Ok(command) = crate::setup::command() {
                ui.monospace(&command);
                if ui.button("Copy setup command").clicked() {
                    ui.ctx().copy_text(command);
                }
            }
        });
    }
}

fn path_input(ui: &mut egui::Ui, value: &mut String) -> egui::Response {
    let response = ui.add(
        egui::TextEdit::singleline(value).desired_width((ui.available_width() - 90.0).max(80.0)),
    );
    if response.has_focus() {
        // TextEdit uses selection.stroke for its focused frame. Overlay only
        // that frame so text selection retains the normal teal semantics.
        ui.painter().rect_stroke(
            response.rect,
            egui::CornerRadius::same(4),
            egui::Stroke::new(2.0, egui::Color32::WHITE),
            egui::StrokeKind::Inside,
        );
    }
    response
}

fn queue_scene(state: &mut DisplayState, action: scene::SceneAction) {
    if let Some(view) = &state.presented {
        state.scene_result = None;
        state.pending_scene = Some(SceneRequest {
            frame: view.identity.clone(),
            action,
        });
    }
}
fn diagnostics(
    ui: &mut egui::Ui,
    state: &DisplayState,
    message: Option<&str>,
    pointer_cut: Option<viewport::PointerCut>,
) {
    egui::CollapsingHeader::new("Diagnostics").show(ui, |ui| {
        ui.monospace(format!(
            "Boundary {} | Generation {} | Actuation {:?}: {} products, {} controls",
            state.boundary,
            state.generation,
            state.actuation_boundary,
            state.actuation_products,
            state.controls.len()
        ));
        if let Some(cut) = pointer_cut {
            ui.monospace(format!(
                "Primary pressed={} released={} held={} begin={} end={}; worker drag {}",
                cut.pressed, cut.released, cut.held, cut.started, cut.stopped, state.dragging
            ));
        }
        egui::ScrollArea::vertical()
            .max_height(90.0)
            .show(ui, |ui| {
                if let Some(error) = state
                    .error
                    .as_ref()
                    .map(|notice| notice.details.as_str())
                    .or(message)
                {
                    ui.label(error);
                }
            });
    });
}

fn free_ancestor(bodies: &[NativeBody], selected: usize) -> Option<usize> {
    let mut parent = bodies.get(selected)?.parent;
    for _ in 0..bodies.len() {
        let id = parent?;
        let body = bodies.get(id)?;
        if body.mobility == BodyMobility::FreeJoint {
            return Some(id);
        }
        parent = body.parent;
    }
    None
}

fn inspector(ui: &mut egui::Ui, state: &mut DisplayState, list_height: f32) {
    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);
    ui.strong("Scene");
    if state.bodies.is_empty() {
        ui.weak("No native bodies loaded");
    }
    egui::ScrollArea::vertical()
        .max_height(list_height)
        .id_salt("native_bodies")
        .show(ui, |ui| {
            let bodies = state.bodies.clone();
            for body in bodies.iter() {
                let name = if body.name.is_empty() {
                    format!("Body {}", body.id)
                } else {
                    body.name.clone()
                };
                let mut depth = 0;
                let mut parent = body.parent;
                while let Some(id) = parent {
                    depth += 1;
                    parent = bodies.get(id).and_then(|body| body.parent);
                    if depth >= 16 {
                        break;
                    }
                }
                let label = format!("{}{}", "  ".repeat(depth), name);
                if ui
                    .selectable_label(
                        state
                            .selection
                            .as_ref()
                            .is_some_and(|hit| hit.body == body.id),
                        label,
                    )
                    .on_hover_text(format!(
                        "{}\nNative body {} · {}",
                        body.name,
                        body.id,
                        body.mobility.label()
                    ))
                    .clicked()
                {
                    queue_scene(state, scene::SceneAction::SelectBody(body.id));
                }
            }
        });
    ui.separator();
    ui.strong("Selected body");
    let mut selected_ancestor = None;
    if let Some(hit) = &state.selection {
        if let Some(body) = state.bodies.get(hit.body) {
            ui.label(if body.name.is_empty() {
                "Unnamed body"
            } else {
                &body.name
            })
            .on_hover_text(&body.name);
            ui.weak(format!(
                "Native body {} · {}",
                body.id,
                body.mobility.label()
            )).on_hover_text(match body.mobility {
                BodyMobility::Fixed => "Fixed bodies are selectable but cannot be manipulated.",
                BodyMobility::FreeJoint => "Running force, or paused translation when this free body has no active weld/connect constraint.",
                BodyMobility::Articulated => "Running physical force only; paused pose edits require the selected body's own free joint.",
            });
            if let Some(ancestor) = free_ancestor(&state.bodies, body.id) {
                let name = &state.bodies[ancestor].name;
                if ui
                    .small_button(format!("Select movable ancestor: {name}"))
                    .clicked()
                {
                    selected_ancestor = Some(ancestor);
                }
            }
            ui.weak(format!(
                "Geom {}",
                hit.geom
                    .map(|id| id.to_string())
                    .unwrap_or_else(|| "none".into())
            ));
            ui.monospace(format!(
                "Hit [{:.3}, {:.3}, {:.3}] m",
                hit.point[0], hit.point[1], hit.point[2]
            ));
            if let Some(view) = &state.presented {
                let identity = view.identity.epoch.model.to_hex();
                ui.small(format!("Model {}", &identity[..12]))
                    .on_hover_text(identity);
            } else {
                ui.small("Model unavailable");
            }
        }
        if let Some(ancestor) = selected_ancestor {
            queue_scene(state, scene::SceneAction::SelectBody(ancestor));
        }
        if ui.small_button("Clear selection").clicked() {
            queue_scene(state, scene::SceneAction::Clear);
        }
    } else {
        ui.weak("Click a surface or choose a native body.");
    }
}
