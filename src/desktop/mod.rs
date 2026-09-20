//! Desktop presentation. Only the worker owns simulation state and authority.

mod viewport;

use crate::config::Options;
use crate::mujoco::{RenderedCamera, ViewCamera};
use eframe::egui;
use std::sync::{
    Arc, Mutex,
    mpsc::{self, Receiver, SyncSender},
};

#[derive(Clone, Copy, Debug)]
pub(super) enum Command {
    Run,
    Pause,
    Step,
    Reset,
    Stop,
}

#[derive(Default)]
pub(super) struct DisplayState {
    pub(super) boundary: u64,
    pub(super) time_seconds: f64,
    pub(super) generation: u64,
    pub(super) running: bool,
    pub(super) ready: bool,
    pub(super) finished: bool,
    pub(super) error: Option<String>,
    pub(super) frame: Option<RenderedCamera>,
    pub(super) camera: Option<ViewCamera>,
    pub(super) pending_camera: Option<ViewCamera>,
    pub(super) controls: Vec<f64>,
    pub(super) actuation_boundary: Option<u64>,
    pub(super) actuation_products: usize,
}

pub(super) struct Worker {
    pub(super) commands: Receiver<Command>,
    pub(super) display: Arc<Mutex<DisplayState>>,
}

pub(super) fn run(options: Options) -> Result<(), String> {
    let (commands, receiver) = mpsc::sync_channel(8);
    let display = Arc::new(Mutex::new(DisplayState::default()));
    let worker = Worker {
        commands: receiver,
        display: display.clone(),
    };
    let worker_display = display.clone();
    let thread = std::thread::Builder::new()
        .name("simulation".into())
        .spawn(move || {
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                tokio::runtime::Builder::new_multi_thread()
                    .worker_threads(1)
                    .enable_all()
                    .build()
                    .map_err(|e| e.to_string())
                    .and_then(|runtime| {
                        runtime.block_on(crate::runtime::run(options, Some(worker)))
                    })
            }))
            .unwrap_or_else(|_| Err("simulation worker panicked".into()));
            if let Ok(mut state) = worker_display.lock() {
                state.running = false;
                state.finished = true;
                if let Err(error) = &outcome {
                    state.error = Some(error.clone());
                }
            }
            outcome
        })
        .map_err(|e| e.to_string())?;
    let shutdown = commands.clone();
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 820.0])
            .with_min_inner_size([800.0, 560.0]),
        ..Default::default()
    };
    let gui = eframe::run_native(
        "Phoxal Simulator",
        native_options,
        Box::new(move |cc| {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            Ok(Box::new(Desktop {
                commands,
                display,
                texture: None,
                message: None,
            }))
        }),
    )
    .map_err(|e| e.to_string());
    // Dropping the last sender also stops the worker if the finite queue is full.
    let _ = shutdown.try_send(Command::Stop);
    drop(shutdown);
    let simulation = thread
        .join()
        .map_err(|_| "simulation worker panicked".to_owned())?;
    gui.and(simulation)
}

struct Desktop {
    commands: SyncSender<Command>,
    display: Arc<Mutex<DisplayState>>,
    texture: Option<egui::TextureHandle>,
    message: Option<String>,
}

impl Desktop {
    fn send(&mut self, command: Command) {
        self.message = self
            .commands
            .try_send(command)
            .err()
            .map(|e| format!("Control unavailable: {e}"));
    }
}

impl eframe::App for Desktop {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        ui.ctx()
            .request_repaint_after(std::time::Duration::from_millis(33));
        let display = self.display.clone();
        let Ok(mut state) = display.lock() else {
            ui.label("Simulation state is unavailable");
            return;
        };
        if let Some(frame) = state.frame.take() {
            let image = egui::ColorImage::from_rgb(frame.resolution(), frame.rgb());
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
            .inner_margin(24)
            .show(ui, |ui| {
                ui.style_mut().spacing.item_spacing = egui::vec2(12.0, 12.0);
                ui.style_mut().spacing.button_padding = egui::vec2(16.0, 10.0);
                ui.visuals_mut().override_text_color = Some(egui::Color32::from_rgb(225, 232, 241));
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("Phoxal Simulator").size(24.0).strong());
                    ui.separator();
                    let status = if state.error.is_some() {
                        "Failed"
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
                ui.add_space(12.0);
                ui.horizontal(|ui| {
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
                    ui.separator();
                    ui.monospace(format!(
                        "{:.3} s   |   Boundary {}   |   Generation {}",
                        state.time_seconds, state.boundary, state.generation
                    ));
                });
                if let Some(error) = state.error.as_ref().or(self.message.as_ref()) {
                    ui.colored_label(egui::Color32::LIGHT_RED, error);
                }
                ui.add_space(12.0);
                if let Some(texture) = &self.texture {
                    if let Some(view) = viewport::show(ui, texture, state.camera) {
                        state.pending_camera = Some(view);
                    }
                } else {
                    ui.centered_and_justified(|ui| {
                        ui.spinner();
                        ui.label("Preparing native scene…");
                    });
                }
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    ui.weak("Drag to orbit · Scroll to zoom");
                    ui.separator();
                    if let Some(boundary) = state.actuation_boundary {
                        ui.weak(format!(
                            "Actuation at {boundary}: {} products, {} controls",
                            state.actuation_products,
                            state.controls.len()
                        ));
                    }
                });
            });
    }
}
