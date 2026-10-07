//! Desktop presentation. Only the worker owns simulation state and authority.

pub(super) mod scene;
pub(super) mod viewport;

#[cfg(test)]
mod tests;

use crate::config::Options;
use crate::execution::Pacing;
use crate::mujoco::{BodyMobility, NativeBody, NativeSelection, ViewCamera};
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
    pub(super) error: Option<String>,
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

pub(super) struct Worker {
    pub(super) commands: Receiver<Command>,
    pub(super) display: Arc<Mutex<DisplayState>>,
}

struct Control {
    commands: Sender<Command>,
    thread:
        Option<std::thread::JoinHandle<Result<Option<crate::runtime::TerminalEvidence>, String>>>,
}

fn start_worker(options: Options, display: Arc<Mutex<DisplayState>>) -> Result<Control, String> {
    let (commands, receiver) = mpsc::channel(8);
    let worker = Worker {
        commands: receiver,
        display: display.clone(),
    };
    let thread = std::thread::Builder::new()
        .name("simulation".into())
        .spawn(move || {
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                crate::lifecycle::run_owned(options, Some(worker))
            }))
            .unwrap_or_else(|_| Err("simulation worker panicked".into()));
            // Finished means native authority and every owned process have been released.
            if let Ok(mut state) = display.lock() {
                state.running = false;
                state.finished = true;
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
                    state.error = Some(error.clone());
                }
            }
            outcome
        })
        .map_err(|e| e.to_string())?;
    Ok(Control {
        commands,
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
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
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
    // Closing the command channel also stops the run if the finite queue is full.
    let (disconnected, _) = mpsc::channel(1);
    owner.commands = disconnected;
    let thread = owner.thread.take();
    drop(owner);
    let simulation = match thread {
        Some(thread) => thread
            .join()
            .map_err(|_| "simulation worker panicked".to_owned())?,
        None => Ok(None),
    };
    gui.and(simulation)
}

struct Desktop {
    control: Arc<Mutex<Control>>,
    display: Arc<Mutex<DisplayState>>,
    options: Option<Options>,
    build_path: String,
    scene_path: String,
    restart: u64,
    texture: Option<egui::TextureHandle>,
    message: Option<String>,
    view_camera: Option<ViewCamera>,
    gesture: Option<(u64, scene::SceneEpoch, egui::Pos2, f32)>,
    next_gesture: u64,
    pointer_cut: Option<viewport::PointerCut>,
    #[cfg(test)]
    last_viewport: Option<egui::Rect>,
}

impl Desktop {
    fn send(&mut self, command: Command) {
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
            .err();
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
        let Ok(mut state) = display.lock() else {
            ui.label("Simulation state is unavailable");
            return;
        };
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
            .fill(egui::Color32::from_rgb(18, 22, 29))
            .inner_margin(12)
            .show(ui, |ui| {
                ui.style_mut().spacing.item_spacing = egui::vec2(8.0, 8.0);
                ui.style_mut().spacing.button_padding = egui::vec2(10.0, 6.0);
                ui.visuals_mut().override_text_color = Some(egui::Color32::from_rgb(225, 232, 241));
                ui.horizontal_wrapped(|ui| {
                    ui.label(egui::RichText::new("Phoxal Simulator").size(18.0).strong());
                    if !state.robot_name.is_empty() { ui.label(&state.robot_name); }
                    ui.separator();
                    let status = if state.error.is_some() {
                        "Failed"
                    } else if self.options.is_none() {
                        "Idle"
                    } else if state.finished {
                        "Stopped"
                    } else if !state.ready {
                        "Connecting"
                    } else if state.running {
                        "Running"
                    } else {
                        "Paused"
                    };
                    ui.label(egui::RichText::new(status).color(if state.error.is_some() {
                        egui::Color32::LIGHT_RED
                    } else {
                        egui::Color32::from_rgb(96, 215, 182)
                    }));
                });
                if state.finished {
                    ui.add_space(16.0);
                    ui.label("Robot build directory");
                    ui.add(egui::TextEdit::singleline(&mut self.build_path).desired_width(f32::INFINITY));
                    ui.label("Scene file");
                    ui.add(egui::TextEdit::singleline(&mut self.scene_path).desired_width(f32::INFINITY));
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
                        let result = (|| -> Result<(), String> {
                            let mut owner = self.control.lock().map_err(|_| "desktop control lock poisoned")?;
                            if let Some(thread) = owner.thread.take() {
                                // The previous execution has already joined all child processes.
                                let _ = thread.join().map_err(|_| "simulation worker panicked")?;
                            }
                            *owner = start_worker(options.clone(), self.display.clone())?;
                            self.options = Some(options);
                            *state = DisplayState::default();
                            self.texture = None;
                            self.view_camera = None;
                            Ok(())
                        })();
                        self.message = result.err();
                    }
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
                                .clicked()
                            {
                                self.send(Command::Step);
                            }
                            if ui
                                .add_enabled(!state.running, egui::Button::new("Reset"))
                                .clicked()
                            {
                                self.send(Command::Reset);
                            }
                            if ui.button("Stop").clicked() {
                                self.send(Command::Stop);
                            }
                        },
                    );
                    if state.finished && self.options.is_some() && ui.button("Restart").clicked() {
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
                            *state = DisplayState::default();
                            self.texture = None;
                            self.view_camera = None;
                            *owner = start_worker(options, self.display.clone())?;
                            Ok(())
                        })();
                        self.message = result.err();
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
                if let Some(error) = state.error.as_ref().or(self.message.as_ref()) {
                    ui.colored_label(egui::Color32::LIGHT_RED, "Simulation / control notice").on_hover_text(error);
                }
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
                            diagnostics(ui, &state, self.message.as_deref(), self.pointer_cut);
                            egui::CollapsingHeader::new("Scene and selected body")
                                .show(ui, |ui| { inspector(ui, &mut state, 140.0); });
                        });
                    self.viewport(ui, &mut state);
                } else {
                    diagnostics(ui, &state, self.message.as_deref(), self.pointer_cut);
                    ui.horizontal_top(|ui| {
                        let height = ui.available_height().max(1.0);
                        ui.allocate_ui_with_layout(egui::vec2(220.0,height), egui::Layout::top_down(egui::Align::Min), |ui| {
                            egui::ScrollArea::vertical().id_salt("wide_scene_inspector")
                                .max_height(height).show(ui, |ui| {
                                    inspector(ui, &mut state, (height-180.0).max(60.0));
                                });
                        });
                        ui.separator();
                        self.viewport(ui, &mut state);
                    });
                }
            });
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
            ui.colored_label(egui::Color32::LIGHT_RED, error);
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
                if let Some(error) = state.error.as_deref().or(message) {
                    ui.label(error);
                }
            });
    });
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
                if ui
                    .selectable_label(
                        state
                            .selection
                            .as_ref()
                            .is_some_and(|hit| hit.body == body.id),
                        name,
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
        if ui.small_button("Clear selection").clicked() {
            queue_scene(state, scene::SceneAction::Clear);
        }
    } else {
        ui.weak("Click a surface or choose a native body.");
    }
}
