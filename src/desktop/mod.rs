//! Scene-first desktop. Authored/native/execution ownership belongs to application.
use crate::app::scene;
mod controls;
#[cfg(all(test, external_source_qualification))]
mod external_qualification;
mod follow;
mod icons;
mod modal;
#[cfg(test)]
mod tests;
mod theme;
pub(super) mod viewport;
use crate::app::world::{Edit, Handle};
use crate::app::{Command, RunOutcome, RunState};
use crate::config::Options;
use crate::document::{Departure, PendingAction};
use crate::execution::Pacing;
use crate::mujoco::{BodyMobility, NativeBody, ViewCamera};
use crate::notice::Notice;
use eframe::egui;
use scene::{CameraRequest, SceneRequest};
use std::sync::{Arc, Mutex};
#[cfg(all(test, native_qualification))]
use tokio::sync::mpsc;

pub(super) fn run(options: Options) -> Result<RunOutcome, String> {
    open(Some(options))
}
pub(super) fn idle() -> Result<RunOutcome, String> {
    open(None)
}
fn open(options: Option<Options>) -> Result<RunOutcome, String> {
    let world = Arc::new(Mutex::new(Handle::start(options)?));
    let shutdown = world.clone();
    let display = world
        .lock()
        .map_err(|_| "World lock poisoned")?
        .display
        .clone();
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 820.0])
            .with_min_inner_size([320.0, 420.0]),
        ..Default::default()
    };
    eprintln!("phoxal-simulator: creating native scene-first window");
    let gui = eframe::run_native(
        "Phoxal Simulator",
        native_options,
        Box::new(move |cc| {
            theme::apply(&cc.egui_ctx);
            eprintln!("phoxal-simulator: native application initialized");
            Ok(Box::new(Desktop::new(world, display)))
        }),
    )
    .map_err(|e| e.to_string());
    eprintln!("phoxal-simulator: eframe returned ok={}", gui.is_ok());
    let simulation = shutdown
        .lock()
        .map_err(|_| "World lock poisoned")?
        .shutdown();
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
fn yaw_orientation(base: [f64; 4], degrees: f64) -> [f64; 4] {
    let (s, c) = (degrees.to_radians() / 2.).sin_cos();
    let [w, x, y, z] = base;
    [w * c - z * s, x * c + y * s, y * c - x * s, z * c + w * s]
}
struct Desktop {
    world: Arc<Mutex<Handle>>,
    display: Arc<Mutex<RunState>>,
    texture: Option<egui::TextureHandle>,
    candidate_texture: Option<(String, egui::TextureHandle)>,
    sheet_bounds: Option<egui::Rect>,
    status_bounds: Option<egui::Rect>,
    import_dirty: bool,
    message: Option<Notice>,
    view_camera: Option<ViewCamera>,
    gesture: Option<(u64, scene::SceneEpoch, egui::Pos2, f32)>,
    next_gesture: u64,
    pointer_cut: Option<viewport::PointerCut>,
    departure: Departure,
    modal_focus: modal::Focus,
    allow_close: bool,
    inspector_open: bool,
    console_open: bool,
    console_height: f32,
    manipulate: bool,
    follow: Option<follow::Target>,
    help_open: bool,
    outline_open: bool,
    motion_tab: bool,
    larger_targets: bool,
    captured: Vec<egui::Rect>,
    import_open: bool,
    import_cancel_requested: bool,
    placement_pending: bool,
    selection_pending: Option<scene::FrameIdentity>,
    import_build: Option<std::path::PathBuf>,
    import_target: String,
    import_position: [f64; 3],
    import_quaternion: [f64; 4],
    import_yaw: f64,
    #[cfg(test)]
    last_viewport: Option<egui::Rect>,
    #[cfg(test)]
    last_follow: Option<egui::Rect>,
}
impl Desktop {
    fn new(world: Arc<Mutex<Handle>>, display: Arc<Mutex<RunState>>) -> Self {
        Self {
            world,
            display,
            texture: None,
            candidate_texture: None,
            sheet_bounds: None,
            status_bounds: None,
            import_dirty: false,
            message: None,
            view_camera: None,
            gesture: None,
            next_gesture: 0,
            pointer_cut: None,
            departure: Default::default(),
            modal_focus: Default::default(),
            allow_close: false,
            inspector_open: false,
            console_open: false,
            console_height: 140.,
            manipulate: false,
            follow: None,
            help_open: false,
            outline_open: false,
            motion_tab: false,
            larger_targets: false,
            captured: Vec::new(),
            import_open: false,
            import_cancel_requested: false,
            placement_pending: false,
            selection_pending: None,
            import_build: None,
            import_target: String::new(),
            import_position: [0.; 3],
            import_quaternion: [1., 0., 0., 0.],
            import_yaw: 0.,
            #[cfg(test)]
            last_viewport: None,
            #[cfg(test)]
            last_follow: None,
        }
    }
    fn send(&mut self, command: Command) {
        let result = self
            .world
            .lock()
            .map_err(|_| "World lock poisoned".to_owned())
            .and_then(|w| w.send(command));
        self.message = result.err().map(Notice::from);
    }
    fn edit(&mut self, edit: Edit) {
        let result = self
            .world
            .lock()
            .map_err(|_| "World lock poisoned".to_owned())
            .and_then(|w| w.edit(edit));
        self.message = result.err().map(Notice::from);
    }
    fn leave(&mut self, action: PendingAction, state: &RunState) {
        if matches!(action, PendingAction::Close) && !state.finished && !state.stopping {
            // Close requests cancellation of owned execution work first;
            // the departure proceeds once cleanup is confirmed.
            self.send(Command::Stop);
        }
        self.departure.begin(action);
    }
    fn draw(&mut self, ui: &mut egui::Ui) {
        ui.spacing_mut().item_spacing.y = 0.;
        ui.ctx()
            .request_repaint_after(std::time::Duration::from_millis(33));
        let display = self.display.clone();
        let Ok(mut shared) = display.lock() else {
            ui.label("World state unavailable");
            return;
        };
        let mut state = shared.observe();
        drop(shared);
        if ui.ctx().input(|i| i.viewport().close_requested()) && !self.allow_close {
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::CancelClose);
            if self.departure.pending.is_none() {
                eprintln!("phoxal-simulator: native close received");
                self.leave(PendingAction::Close, &state);
            }
        }
        // Any open modal owner (placement sheet, help) owns the whole
        // background: header, menu, panels, console,
        // footer and toolbar actions are disabled for pointer, keyboard and
        // accessible input alike, not merely covered by a pointer backdrop.
        // The modals themselves draw in their own areas and stay live.
        let modal_owner = self.import_open || self.help_open;
        if modal_owner {
            // Begin the modal focus owner before the disabled background
            // drops focus, so the invoking control is recorded as the
            // restore target on the entering frame. The per-surface begin
            // calls later in the frame are idempotent for the same owner.
            let owner = if self.import_open {
                egui::Id::new("import")
            } else {
                egui::Id::new("help")
            };
            self.modal_focus.begin(ui.ctx(), owner);
            ui.disable();
        }
        if self.departure.pending.is_some() && !state.finished && !state.stopping {
            self.send(Command::Stop);
        }
        if let Some(action) = self
            .departure
            .take_ready(state.finished && !state.document.busy, state.cleanup_failed)
        {
            match action {
                PendingAction::New => self.edit(Edit::New),
                PendingAction::OpenScene(p) => self.edit(Edit::Scene(p)),
                PendingAction::Close => {
                    self.allow_close = true;
                    eprintln!("phoxal-simulator: application close accepted");
                    ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
                }
            }
        }
        if let Some(result) = &state.scene_result
            && self.selection_pending.as_ref() == Some(&result.frame)
        {
            if !result.stale && state.selection.is_some() {
                self.inspector_open = true;
                if ui.available_width() < 600. {
                    self.outline_open = false;
                }
            }
            self.selection_pending = None;
        }
        if self.placement_pending && !state.document.busy {
            self.placement_pending = false;
            if state.document.robot.is_some() && state.document_error.is_none() {
                self.inspector_open = true;
                if ui.available_width() < 600. {
                    self.outline_open = false;
                }
            }
        }
        if let Some(result) = state.scene_result.take()
            && state
                .requests
                .presented
                .as_ref()
                .is_some_and(|v| v.identity.epoch == result.frame.epoch)
            && !result.stale
            && let Some(camera) = result.camera
        {
            self.view_camera = Some(camera);
        }
        if let Some(frame) = state.frame.take() {
            if state
                .requests
                .presented
                .as_ref()
                .is_none_or(|v| v.identity.epoch != frame.view.identity.epoch)
            {
                self.view_camera = Some(frame.view.camera);
                self.gesture = None;
            }
            state.requests.presented = Some(frame.view);
            let image = egui::ColorImage::from_rgb(frame.image.resolution(), frame.image.rgb());
            if let Some(t) = &mut self.texture {
                t.set(image, egui::TextureOptions::LINEAR)
            } else {
                self.texture = Some(ui.ctx().load_texture(
                    "scene",
                    image,
                    egui::TextureOptions::LINEAR,
                ));
            }
        }
        if let (Some(target), Some(view), Some(camera)) = (
            &mut self.follow,
            state.requests.presented.as_ref(),
            &mut self.view_camera,
        ) {
            if target.track(view, camera) {
                state.requests.pending_camera = Some(CameraRequest {
                    epoch: view.identity.epoch.clone(),
                    camera: *camera,
                });
            } else {
                self.follow = None;
                self.message = Some(Notice::new(
                    "Follow ended: target or execution changed",
                    "select a valid body and activate Follow",
                    "The last valid camera view is preserved.",
                ));
            }
        }
        if self.import_cancel_requested && !state.document.busy {
            self.edit(Edit::CancelImport);
            self.import_open = false;
            self.import_cancel_requested = false;
        }
        if state.document.robot.is_some() && !state.document.busy {
            self.import_open = false;
        }
        if state.finished {
            self.manipulate = false;
        }
        let stopped = state.finished && !state.cleanup_failed && !state.document.busy;
        let narrow_header = ui.available_width() < 600.;
        let header = egui::Frame::new()
            .fill(theme::SURFACE)
            .inner_margin(egui::Margin::symmetric(
                if narrow_header { 8 } else { 16 },
                8,
            ))
            .show(ui, |ui| {
                let size =
                    controls::control_size(ui).max(if self.larger_targets { 40. } else { 0. });
                let narrow = narrow_header;
                ui.spacing_mut().interact_size.y = size;
                ui.spacing_mut().item_spacing.x = 5.;
                ui.spacing_mut().button_padding = egui::vec2(0., 0.);
                let font = egui::TextStyle::Button.resolve(ui.style());
                let import_label = if narrow_header {
                    "Import"
                } else {
                    "Import robot"
                };
                let import_width = ui
                    .fonts_mut(|f| {
                        f.layout_no_wrap(import_label.into(), font.clone(), theme::TEXT)
                            .size()
                            .x
                    })
                    .max(size)
                    + 18.
                    + icons::GLYPH
                    + 8.;
                ui.horizontal(|ui| {
                    if controls::button(
                        ui,
                        controls::Icon::PanelLeft,
                        "Scene outline",
                        self.outline_open,
                        true,
                        "",
                        false,
                    )
                    .clicked()
                    {
                        self.outline_open = !self.outline_open;
                        if narrow && self.outline_open {
                            self.inspector_open = false;
                        }
                    }
                    let name = if state.document.scene_name.is_empty() {
                        "Default scene"
                    } else {
                        &state.document.scene_name
                    };
                    let label = name.to_owned();
                    let accessible = format!("{name}. Open scene and New simulation");
                    // The scene menu measures its full content: label with
                    // its Modified marker, the 16pt chevron glyph, the gap
                    // between them and padding, so wide headers show the
                    // complete identity and only genuine space pressure
                    // truncates against the remaining header width.
                    let menu_width = (ui.fonts_mut(|f| {
                        f.layout_no_wrap(label.clone(), font, theme::TEXT).size().x
                    }) + 8.
                        + icons::GLYPH
                        + 12.)
                        .min(ui.available_width() * 0.5)
                        .max(40.);
                    ui.allocate_ui_with_layout(
                        egui::vec2(menu_width, size),
                        egui::Layout::left_to_right(egui::Align::Center),
                        |ui| {
                            let (response, menu_popup) =
                                egui::containers::menu::MenuButton::from_button(
                                    egui::Button::new((
                                        label,
                                        icons::Icon::ChevronDown
                                            .image(theme::MUTED)
                                            .fit_to_exact_size(egui::vec2(
                                                icons::GLYPH,
                                                icons::GLYPH,
                                            )),
                                    ))
                                    .truncate()
                                    .min_size(egui::vec2(menu_width, size)),
                                )
                                .ui(ui, |ui| {
                                    if ui
                                        .add(egui::Button::image_and_text(
                                            icons::Icon::House
                                                .image(theme::MUTED)
                                                .fit_to_exact_size(egui::vec2(
                                                    icons::GLYPH,
                                                    icons::GLYPH,
                                                )),
                                            "New simulation",
                                        ))
                                        .clicked()
                                    {
                                        self.leave(PendingAction::New, &state);
                                        ui.close();
                                    }
                                    if ui
                                        .add(egui::Button::image_and_text(
                                            icons::Icon::FolderOpen
                                                .image(theme::MUTED)
                                                .fit_to_exact_size(egui::vec2(
                                                    icons::GLYPH,
                                                    icons::GLYPH,
                                                )),
                                            "Open scene…",
                                        ))
                                        .clicked()
                                    {
                                        if let Some(p) = rfd::FileDialog::new()
                                            .add_filter("MuJoCo scene", &["xml"])
                                            .pick_file()
                                        {
                                            self.leave(PendingAction::OpenScene(p), &state);
                                        }
                                        ui.close();
                                    }
                                    ui.separator();
                                });
                            #[cfg(test)]
                            ui.ctx().data_mut(|d| {
                                d.insert_temp(egui::Id::new("document-menu-probe"), response.rect);
                                d.insert_temp(egui::Id::new("document-menu-id-probe"), response.id)
                            });
                            response.widget_info(|| {
                                egui::WidgetInfo::labeled(
                                    egui::WidgetType::Button,
                                    true,
                                    &accessible,
                                )
                            });
                            if response.has_focus()
                                && menu_popup.is_none()
                                && !egui::Popup::is_any_open(ui.ctx())
                            {
                                egui::Tooltip::for_widget(&response).show(|ui| {
                                    ui.label(&accessible);
                                });
                            }
                            if menu_popup.is_none() {
                                response.on_hover_text(accessible);
                            }
                        },
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if controls::button(
                            ui,
                            controls::Icon::PanelRight,
                            "Inspector",
                            self.inspector_open,
                            self.inspector_open
                                || state.selection.is_some()
                                || state.document.robot.is_some(),
                            "Select a native body",
                            false,
                        )
                        .clicked()
                        {
                            self.inspector_open = !self.inspector_open;
                            if narrow && self.inspector_open {
                                self.outline_open = false;
                            }
                        }
                        let import_enabled = stopped && state.document.robot.is_none();
                        // The glyph color stays constant so the widget id is
                        // stable across enabled transitions; egui's disabled
                        // visuals carry the disabled appearance.
                        let import = ui.add_enabled(
                            import_enabled,
                            egui::Button::image_and_text(
                                icons::Icon::Plus
                                    .image(theme::MUTED)
                                    .fit_to_exact_size(egui::vec2(icons::GLYPH, icons::GLYPH)),
                                import_label,
                            )
                            .fill(theme::SURFACE)
                            .stroke(egui::Stroke::new(theme::BORDER_WIDTH, theme::BORDER))
                            .min_size(egui::vec2(import_width, size)),
                        );
                        import.widget_info(|| {
                            egui::WidgetInfo::labeled(
                                egui::WidgetType::Button,
                                import_enabled,
                                "Import robot",
                            )
                        });
                        #[cfg(test)]
                        ui.ctx().data_mut(|d| {
                            d.insert_temp(egui::Id::new("import-button-probe"), import.rect)
                        });
                        if import
                            .on_disabled_hover_text(
                                "Confirm Stop and remove the existing robot before import",
                            )
                            .clicked()
                            && let Some(p) = rfd::FileDialog::new()
                                .set_title("Choose prepared robot build")
                                .pick_folder()
                        {
                            self.import_build = Some(p);
                            self.import_open = true;
                            if let Some(a) = state.document.attachments.first() {
                                self.import_target = a.0.clone();
                                self.import_position = a.1;
                                self.import_quaternion = a.2;
                                self.import_yaw = 0.;
                            } else {
                                self.import_target.clear();
                            }
                        }
                    });
                });
            });
        ui.painter().hline(
            header.response.rect.x_range(),
            header.response.rect.bottom(),
            egui::Stroke::new(theme::BORDER_WIDTH, theme::LINE),
        );
        if state.document.cancelable && !self.import_open {
            egui::Area::new(egui::Id::new("admission-cancel"))
                .anchor(egui::Align2::CENTER_TOP, egui::vec2(0., 64.))
                .show(ui.ctx(), |ui| {
                    ui.label("Admitting scene…");
                    if ui.button("Cancel load").clicked()
                        && let Ok(world) = self.world.lock()
                    {
                        world.cancel_admission();
                    }
                });
        }
        let status = if state.cleanup_failed {
            "Cleanup incomplete"
        } else if state.stopping {
            "Stopping"
        } else if state.error.is_some() {
            "Failed"
        } else if state.document.busy || !state.ready {
            if state.phase.label().is_empty() {
                "Preparing"
            } else {
                state.phase.label()
            }
        } else if state.finished {
            if matches!(
                state.completion,
                Some(
                    crate::app::Completion::Stopped
                        | crate::app::Completion::Completed
                        | crate::app::Completion::Cancelled
                )
            ) {
                "Stopped"
            } else {
                "Ready"
            }
        } else if state.running {
            "Running"
        } else {
            "Paused"
        };
        // Contextual panels are closed by default. Notices never consume canvas space.
        let available = ui.available_size();
        let narrow_panel =
            available.x < 600. && (self.outline_open || self.inspector_open || self.import_open);
        let drawer = (available.y * 0.2).min(self.console_height).max(40.);
        let panel_height = if narrow_panel {
            if self.import_open {
                // The placement sheet owns a working share of the stacked
                // canvas so its actions stay reachable; the world preview
                // keeps the remainder above it.
                (available.y * 0.45).clamp(160., 320.)
            } else if self.console_open {
                drawer * 0.5
            } else {
                drawer
            }
        } else {
            0.
        };
        let console_height = if self.console_open {
            if narrow_panel {
                drawer * 0.5
            } else {
                self.console_height.min((available.y * 0.3).max(40.))
            }
        } else {
            0.
        };
        let size = controls::control_size(ui).max(if self.larger_targets { 40. } else { 0. });
        let mode_text = if self.manipulate {
            "Manipulate · physical interaction"
        } else {
            "Navigate · camera only"
        };
        let footer_target = size.max(28.);
        let (mode_width, sim_width, console_width) = ui.fonts_mut(|f| {
            (
                f.layout_no_wrap(
                    mode_text.to_owned(),
                    egui::FontId::proportional(11.),
                    theme::MUTED,
                )
                .size()
                .x,
                f.layout_no_wrap(
                    "Sim 0000:00 · 00.00x".to_owned(),
                    egui::FontId::monospace(11.),
                    theme::TEXT,
                )
                .size()
                .x,
                f.layout_no_wrap(
                    "Console".to_owned(),
                    egui::FontId::proportional(12.),
                    theme::TEXT,
                )
                .size()
                .x + icons::GLYPH
                    + 8.,
            )
        });
        let footer_right = sim_width + 14. + console_width + 12. + 14.;
        let footer_single_row = mode_width + footer_right + 24. <= available.x;
        let footer_height = if footer_single_row {
            footer_target + 12. + theme::BORDER_WIDTH
        } else {
            footer_target + 11. * 1.5 + 12. + 6. + theme::BORDER_WIDTH
        };
        let height = (available.y - panel_height - console_height - footer_height).max(1.);
        let canvas = egui::Rect::from_min_size(ui.cursor().min, egui::vec2(available.x, height));
        let status_height = size + 2. * (theme::PADDING + theme::BORDER_WIDTH);
        let padding = 2. * (theme::PADDING + theme::BORDER_WIDTH);
        let mut camera_width =
            6. * size + 6. * theme::GAP + 6. + 2. * (theme::PADDING + theme::BORDER_WIDTH);
        let mut transport_width =
            4. * size + 5. * theme::GAP + 6. + 84. + 2. * (theme::PADDING + theme::BORDER_WIDTH);
        let hide_toolbars = self.import_open;
        let wrap = camera_width.max(transport_width) > canvas.width() - 2. * theme::INSET;
        let row_height = if wrap {
            size * 2. + theme::GAP + padding
        } else {
            status_height
        };
        if wrap {
            camera_width = 4. * size + 3. * theme::GAP + padding;
            transport_width = camera_width;
        }
        let stack = canvas.width() < camera_width + transport_width + 3. * theme::INSET;
        let transport_rect = egui::Rect::from_min_size(
            egui::pos2(
                canvas.right() - theme::INSET - transport_width,
                canvas.bottom() - theme::INSET - row_height,
            ),
            egui::vec2(transport_width, row_height),
        );
        let camera_rect = egui::Rect::from_min_size(
            egui::pos2(
                canvas.left() + theme::INSET,
                canvas.bottom()
                    - theme::INSET
                    - row_height
                    - if stack {
                        row_height + theme::STACK_GAP
                    } else {
                        0.
                    },
            ),
            egui::vec2(camera_width, row_height),
        );
        let previous = self.captured.split_off(self.captured.len().min(2));
        self.captured = vec![transport_rect, camera_rect];
        self.captured.extend(previous);
        ui.allocate_ui_with_layout(
            canvas.size(),
            egui::Layout::top_down(egui::Align::Min),
            |ui| self.viewport(ui, &mut state),
        );
        let below_panel = if narrow_panel {
            // Reserve the strip at its exact height through a fixed-size
            // allocation, whether or not content paints into it this
            // frame, so the narrow import sheet and footer keep their
            // reserved regions.
            let strip = egui::Rect::from_min_size(
                ui.cursor().min,
                egui::vec2(canvas.width(), panel_height.max(1.)),
            );
            #[cfg(test)]
            ui.ctx()
                .data_mut(|d| d.insert_temp(egui::Id::new("context-strip-probe"), strip));
            egui::Resize::default()
                .fixed_size([canvas.width(), panel_height.max(1.)])
                .show(ui, |ui| {
                    if !self.import_open {
                        // The context owner paints its entire reserved strip,
                        // including space below short scrolled content.
                        ui.painter().rect_filled(strip, 0., theme::SURFACE);
                        // The narrow import sheet paints into this reserved
                        // space after presentation identity is settled;
                        // panels keep the scrolling context.
                        egui::Frame::new()
                            .fill(theme::SURFACE)
                            .inner_margin(6)
                            .show(ui, |ui| {
                                context_scroll(ui, (panel_height - 12.).max(1.), |ui| {
                                    self.context_panel(ui, &mut state, stopped)
                                });
                            });
                    }
                });
            Some(strip)
        } else {
            None
        };
        if self.console_open {
            // The drawer occupies exactly its reserved height, whether the
            // diagnostics content is shorter or longer, so the footer stays
            // anchored at the window bottom without abandoned space.
            let drawer_width = ui.available_width();
            egui::Resize::default()
                .fixed_size([drawer_width, console_height.max(6.)])
                .show(ui, |ui| {
                    let (handle, response) =
                        ui.allocate_exact_size(egui::vec2(drawer_width, 6.), egui::Sense::drag());
                    ui.painter().line_segment(
                        [handle.left_center(), handle.right_center()],
                        egui::Stroke::new(1., theme::BORDER),
                    );
                    if response.dragged() {
                        self.console_height = (self.console_height
                            - ui.input(|i| i.pointer.delta().y))
                        .clamp(60., 300.);
                    }
                    response
                        .on_hover_cursor(egui::CursorIcon::ResizeVertical)
                        .on_hover_text("Resize console");
                    egui::ScrollArea::vertical()
                        .min_scrolled_height(0.)
                        .max_height((console_height - 6.).max(1.))
                        .show(ui, |ui| {
                            diagnostics(
                                ui,
                                &state,
                                self.message.as_ref().map(|n| n.details.as_str()),
                                self.pointer_cut,
                            );
                            notice_details(ui, &state, self.message.as_ref());
                        });
                });
        }
        let footer = ui.allocate_ui_with_layout(
            egui::vec2(available.x, footer_height),
            egui::Layout::left_to_right(egui::Align::Center),
            |ui| {
                ui.painter().hline(
                    ui.max_rect().x_range(),
                    ui.max_rect().top(),
                    egui::Stroke::new(theme::BORDER_WIDTH, theme::LINE),
                );
                egui::Frame::new()
                    .fill(theme::SURFACE)
                    .inner_margin(egui::Margin::symmetric(12, 6))
                    .show(ui, |ui| {
                        ui.set_min_height(ui.available_height());
                        ui.spacing_mut().item_spacing.x = 14.;
                        if footer_single_row {
                            ui.label(egui::RichText::new(mode_text).small().color(theme::MUTED));
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| self.footer_right(ui, &state),
                            );
                        } else {
                            ui.vertical(|ui| {
                                ui.set_min_height(ui.available_height());
                                ui.spacing_mut().item_spacing.y = 6.;
                                ui.label(
                                    egui::RichText::new(mode_text).small().color(theme::MUTED),
                                );
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| self.footer_right(ui, &state),
                                );
                            });
                        }
                    });
            },
        );
        let footer_rect = footer.response.rect;
        let _ = &footer_rect;
        #[cfg(test)]
        ui.ctx()
            .data_mut(|d| d.insert_temp(egui::Id::new("footer-probe"), footer_rect));
        let status_area = egui::Area::new(egui::Id::new("world-status"))
            .fixed_pos(canvas.min + egui::vec2(16., 16.))
            .show(ui.ctx(), |ui| {
                ui.spacing_mut().interact_size.y = size;
                // Content-width badge: the truthful state and nothing else.
                // Measure with the same Body style the label renders with,
                // add balanced 12pt horizontal padding plus the toolbar chrome.
                let width = ui.fonts_mut(|f| {
                    let font = egui::TextStyle::Body.resolve(ui.style());
                    f.layout_no_wrap(status.to_owned(), font, theme::TEXT)
                        .size()
                        .x
                }) + 24.
                    + 2. * (theme::PADDING + theme::BORDER_WIDTH)
                    + 2.;
                let badge = controls::toolbar(ui, width, |ui| {
                    ui.set_min_height(size);
                    ui.with_layout(
                        egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                        |ui| {
                            ui.add(
                                egui::Label::new(egui::RichText::new(status).color(
                                    if state.running {
                                        theme::SUCCESS
                                    } else {
                                        theme::TEXT
                                    },
                                ))
                                .selectable(false),
                            );
                        },
                    );
                });
                let _ = &badge;
                #[cfg(test)]
                ui.ctx().data_mut(|d| {
                    d.insert_temp(egui::Id::new("status-badge-probe"), badge.response.rect)
                });
            });
        use controls::Icon;
        let enabled =
            state.ready && !state.stopping && !state.cleanup_failed && !state.document.busy;
        let transport = if hide_toolbars {
            None
        } else {
            Some(
                egui::Area::new(egui::Id::new("transport"))
                    .fixed_pos(transport_rect.min)
                    .show(ui.ctx(), |ui| {
                if modal_owner {
                    ui.disable();
                }
                        ui.set_width(transport_width);
                        ui.spacing_mut().interact_size.y = size;
                        controls::toolbar(ui, transport_width, |ui| {
                            let (icon, name) = if state.running {
                                (Icon::Pause, "Pause")
                            } else if state.finished && state.completion.is_some() {
                                (Icon::Play, "Restart")
                            } else {
                                (Icon::Play, "Run")
                            };
                            if controls::button(
                                ui,
                                icon,
                                name,
                                false,
                                enabled,
                                "Wait for admission and confirmed cleanup",
                                true,
                            )
                            .clicked()
                            {
                                self.send(if state.running {
                                    Command::Pause
                                } else {
                                    Command::Run
                                });
                            }
                            if controls::button(
                                ui,
                                Icon::StepForward,
                                "Step",
                                false,
                                enabled && !state.running,
                                "Pause before one ordinary boundary",
                                false,
                            )
                            .clicked()
                            {
                                self.send(Command::Step);
                            }
                            if controls::button(
                                ui,
                                Icon::RotateCcw,
                                "Reset",
                                false,
                                enabled && !state.running,
                                "Pause before restoring authored placement",
                                false,
                            )
                            .clicked()
                            {
                                self.send(Command::Reset);
                            }
                            if controls::button(
                                ui,
                                Icon::Square,
                                "Stop",
                                false,
                                !state.finished && !state.stopping,
                                "No active execution",
                                false,
                            )
                            .clicked()
                            {
                                self.send(Command::Stop);
                                self.follow = None;
                            }
                    ui.separator();
                    ui.add_enabled_ui(
                        !state.finished && !state.stopping && !state.cleanup_failed,
                        |ui| {
                            ui.spacing_mut().interact_size = egui::vec2(84., size);
                            // Genuine chevron glyph without the duplicated
                            // native triangle, through the same menu owner.
                            let (response, _popup) =
                                egui::containers::menu::MenuButton::from_button(
                                    egui::Button::new((
                                        state.pacing.label().to_owned(),
                                        icons::Icon::ChevronDown
                                            .image(theme::MUTED)
                                            .fit_to_exact_size(egui::vec2(14., 14.)),
                                    ))
                                    .min_size(egui::vec2(84., size)),
                                )
                                .ui(ui, |ui| {
                                    for mode in [Pacing::Realtime, Pacing::Fast] {
                                        if ui
                                            .selectable_label(state.pacing == mode, mode.label())
                                            .clicked()
                                        {
                                            self.send(Command::SetPacing(mode));
                                            ui.close();
                                        }
                                    }
                                });
                            let help=format!("Execution pacing: {}. Realtime follows wall time; Fast advances as quickly as possible. Start execution to change pacing; Stop and cleanup uncertainty disable changes.",state.pacing.label());
                            response.widget_info(||egui::WidgetInfo::labeled(egui::WidgetType::ComboBox,response.enabled(),&help));
                            #[cfg(test)] ui.ctx().data_mut(|d|d.insert_temp(egui::Id::new("pacing-probe"),(response.rect,response.id)));
                            if response.has_focus(){response.clone().show_tooltip_text(&help);}
                            response.on_hover_text(&help).on_disabled_hover_text(help);
                        },
                    );
                        });
                    })
            )
        };
        let camera_tools = if hide_toolbars {
            None
        } else {
            Some(
                egui::Area::new(egui::Id::new("camera-tools"))
                    .fixed_pos(camera_rect.min)
                    .show(ui.ctx(), |ui| {
                        if modal_owner {
                            ui.disable();
                        }
                        ui.set_width(camera_width);
                        ui.spacing_mut().interact_size.y = size;
                        controls::toolbar(ui, camera_width, |ui| {
                            if controls::button(
                                ui,
                                Icon::MousePointer2,
                                "Navigate",
                                !self.manipulate,
                                true,
                                "",
                                false,
                            )
                            .clicked()
                            {
                                self.manipulate = false;
                            }
                            if controls::button(
                                ui,
                                Icon::Hand,
                                "Manipulate",
                                self.manipulate,
                                enabled && !state.finished,
                                "Start native execution before manipulation",
                                false,
                            )
                            .clicked()
                            {
                                self.manipulate = true;
                            }
                            ui.separator();
                            if controls::button(
                                ui,
                                Icon::Scan,
                                "Frame selected",
                                false,
                                state.selection.is_some(),
                                "Select a native body",
                                false,
                            )
                            .clicked()
                            {
                                self.follow = None;
                                queue_scene(&mut state, scene::SceneAction::Focus);
                            }
                            let target = follow::candidate(&state);
                            let name = if self.follow.is_some() {
                                "Stop following"
                            } else if state.robot_root.is_some() {
                                "Follow robot"
                            } else {
                                "Follow body"
                            };
                            // New binding needs the presented view the owner
                            // binds against; an already bound target keeps
                            // its Stop action independent of presentation.
                            let presentable = state.requests.presented.is_some();
                            if controls::button(
                                ui,
                                Icon::Crosshair,
                                name,
                                self.follow.is_some(),
                                self.follow.is_some() || (target.is_some() && presentable),
                                if presentable {
                                    "Select a verified robot part or movable native body"
                                } else {
                                    "Wait for the scene presentation to follow"
                                },
                                false,
                            )
                            .clicked()
                            {
                                self.follow = if self.follow.is_some() {
                                    None
                                } else {
                                    target.and_then(|(body, label)| {
                                        state.requests.presented.as_ref().map(|view| {
                                            follow::Target {
                                                epoch: view.identity.epoch.clone(),
                                                body,
                                                label,
                                            }
                                        })
                                    })
                                };
                            }
                            if controls::button(
                                ui,
                                Icon::House,
                                "Default view",
                                false,
                                state.ready,
                                "Wait for native scene",
                                false,
                            )
                            .clicked()
                            {
                                self.follow = None;
                                queue_scene(&mut state, scene::SceneAction::DefaultCamera);
                            }
                            if controls::button(
                                ui,
                                Icon::CircleHelp,
                                "Help",
                                self.help_open,
                                true,
                                "",
                                false,
                            )
                            .clicked()
                            {
                                self.help_open = !self.help_open;
                            }
                        });
                    }),
            )
        };
        if let (Some(transport), Some(camera_tools)) = (&transport, &camera_tools) {
            self.captured = vec![transport.response.rect, camera_tools.response.rect];
        }
        let follow_rect = if let Some(target) = &self.follow {
            let width = if canvas.width() < 600. {
                (canvas.width() * 0.38).min(180.)
            } else {
                260.
            };
            let badge = egui::Area::new(egui::Id::new("follow-target"))
                .fixed_pos(if canvas.width() < 600. {
                    egui::pos2(
                        canvas.left() + theme::INSET,
                        status_area.response.rect.bottom() + theme::STACK_GAP,
                    )
                } else {
                    egui::pos2(
                        canvas.right() - theme::INSET - width,
                        canvas.top() + theme::INSET,
                    )
                })
                .show(ui.ctx(), |ui| {
                    if modal_owner {
                        ui.disable();
                    }
                    egui::Frame::new()
                        .fill(theme::SURFACE)
                        .stroke(egui::Stroke::new(theme::BORDER_WIDTH, theme::BORDER))
                        .inner_margin(theme::PADDING as i8)
                        .corner_radius(theme::RADIUS)
                        .show(ui, |ui| {
                            ui.set_width(width - 12.);
                            let label = format!("Following {}", target.label);
                            ui.add(egui::Label::new(&label).truncate())
                                .on_hover_text(label);
                        });
                });
            Some(badge.response.rect)
        } else {
            None
        };
        #[cfg(test)]
        {
            self.last_follow = follow_rect;
        }
        let upper = match (&transport, &camera_tools) {
            (Some(transport), Some(camera_tools)) => transport
                .response
                .rect
                .top()
                .min(camera_tools.response.rect.top()),
            (Some(only), None) | (None, Some(only)) => only.response.rect.top(),
            (None, None) => canvas.bottom() - theme::INSET,
        };
        let has_notice = state.cleanup_failed
            || state.error.is_some()
            || state.document_error.is_some()
            || self.message.is_some()
            || state.interaction_error.is_some();
        let top_bottom = if canvas.width() < 600. {
            status_area.response.rect.bottom()
        } else {
            follow_rect.map_or(status_area.response.rect.bottom(), |r| {
                r.bottom().max(status_area.response.rect.bottom())
            })
        };
        let notice_height =
            (upper - theme::NOTICE_GAP - top_bottom - theme::STACK_GAP - 22.).clamp(1., 180.);
        let panel_top = top_bottom + theme::STACK_GAP;
        let panel_bottom = upper
            - theme::NOTICE_GAP
            - if has_notice {
                notice_height + 22. + theme::NOTICE_GAP
            } else {
                0.
            };
        if !narrow_panel && self.outline_open {
            let width = 190.;
            let bounds = egui::Rect::from_min_max(
                egui::pos2(canvas.left() + theme::INSET, panel_top),
                egui::pos2(canvas.left() + theme::INSET + width, panel_bottom),
            );
            let panel = egui::Area::new(egui::Id::new("outline-panel"))
                .fixed_pos(bounds.min)
                .show(ui.ctx(), |ui| {
                    if modal_owner {
                        ui.disable();
                    }
                    egui::Resize::default()
                        .fixed_size([width, (panel_bottom - panel_top).max(60.)])
                        .show(ui, |ui| {
                            egui::Frame::new()
                                .fill(theme::SURFACE)
                                .inner_margin(10)
                                .stroke(egui::Stroke::new(theme::BORDER_WIDTH, theme::LINE))
                                .corner_radius(theme::RADIUS)
                                .show(ui, |ui| {
                                    ui.set_min_width(width - 22.);
                                    egui::ScrollArea::vertical()
                                        .min_scrolled_height(0.)
                                        .max_height((panel_bottom - panel_top - 22.).max(1.))
                                        .show(ui, |ui| {
                                            panel_heading(
                                                ui,
                                                "Outline",
                                                "Close outline",
                                                &mut self.outline_open,
                                            );
                                            outline(ui, &mut state);
                                        });
                                });
                        });
                });
            let outline_rect = panel.response.rect.intersect(bounds);
            #[cfg(test)]
            ui.ctx()
                .data_mut(|d| d.insert_temp(egui::Id::new("outline-panel-probe"), outline_rect));
            self.captured.push(outline_rect);
        }
        if !narrow_panel && self.inspector_open {
            let width = 235.;
            let bounds = egui::Rect::from_min_max(
                egui::pos2(canvas.right() - theme::INSET - width, panel_top),
                egui::pos2(canvas.right() - theme::INSET, panel_bottom),
            );
            let panel = egui::Area::new(egui::Id::new("inspector-panel"))
                .fixed_pos(bounds.min)
                .show(ui.ctx(), |ui| {
                    if modal_owner {
                        ui.disable();
                    }
                    egui::Resize::default()
                        .fixed_size([width, (panel_bottom - panel_top).max(60.)])
                        .show(ui, |ui| {
                            egui::Frame::new()
                                .fill(theme::SURFACE)
                                .inner_margin(egui::Margin::symmetric(14, 12))
                                .stroke(egui::Stroke::new(theme::BORDER_WIDTH, theme::LINE))
                                .corner_radius(theme::RADIUS)
                                .show(ui, |ui| {
                                    ui.set_min_width(width - 30.);
                                    context_scroll(ui, (panel_bottom - panel_top - 26.).max(1.), |ui| {
                                            panel_heading(
                                                ui,
                                                "Inspector",
                                                "Close inspector",
                                                &mut self.inspector_open,
                                            );
                                            if state.selection.is_none() {
                                                ui.label(
                                                    egui::RichText::new("Nothing selected")
                                                        .strong(),
                                                );
                                                ui.label(
                                                    egui::RichText::new(
                                                        "Click an object in the scene or choose one in Outline",
                                                    )
                                                    .small()
                                                    .color(theme::MUTED),
                                                );
                                                ui.add_space(6.);
                                            } else {
                                                inspector_header(ui, &state);
                                                inspector_tabs(ui, &mut self.motion_tab);
                                                self.inspector_content(
                                                    ui,
                                                    &mut state,
                                                    self.motion_tab,
                                                );
                                            }
                                            self.robot_actions(ui, &mut state, stopped);
                                        });
                                });
                        });
                });
            let inspector_rect = panel.response.rect.intersect(bounds);
            #[cfg(test)]
            ui.ctx().data_mut(|d| {
                d.insert_temp(egui::Id::new("inspector-panel-probe"), inspector_rect)
            });
            self.captured.push(inspector_rect);
        }
        if let Some(rect) = below_panel {
            self.captured.push(rect);
        }
        if has_notice {
            let max_height = notice_height;
            let area = egui::Area::new(egui::Id::new("notice"))
                .pivot(egui::Align2::RIGHT_BOTTOM)
                .fixed_pos(egui::pos2(
                    canvas.right() - theme::INSET,
                    upper - theme::NOTICE_GAP,
                ))
                .show(ui.ctx(), |ui| {
                    if modal_owner {
                        ui.disable();
                    }
                    egui::Frame::new()
                        .fill(theme::SURFACE)
                        .stroke(egui::Stroke::new(theme::BORDER_WIDTH, theme::LINE))
                        .inner_margin(10)
                        .corner_radius(theme::RADIUS)
                        .show(ui, |ui| {
                            ui.set_width(
                                (if let Some(follow)=follow_rect.filter(|_|canvas.width()<600.) {
                                    canvas.right()
                                        - theme::INSET
                                        - follow.right()
                                        - theme::INSET
                                        - 22.
                                } else {
                                    canvas.width() - 2. * theme::INSET - 22.
                                })
                                .clamp(80., 320.),
                            );
                            ui.visuals_mut().override_text_color = Some(theme::TEXT);
                            egui::ScrollArea::vertical()
                                .min_scrolled_height(0.)
                                .max_height(max_height)
                                .show(ui, |ui| {
                                    notice(ui, &state, self.message.as_ref());
                                    if self.message.is_some()
                                        && ui.button("Dismiss message").clicked()
                                    {
                                        self.message = None;
                                    }
                                });
                        });
                });
            self.captured.push(area.response.rect);
        }
        self.captured.push(status_area.response.rect);
        self.status_bounds = Some(status_area.response.rect);
        if let Some(rect) = follow_rect {
            self.captured.push(rect);
        }
        if self.help_open {
            self.modal_focus.begin(ui.ctx(), egui::Id::new("help"));
            let mut ids = Vec::new();
            let response = egui::Modal::new(egui::Id::new("help")).show(ui.ctx(), |ui| {
                ui.set_width((ui.ctx().content_rect().width()-48.).clamp(200.,420.));
                egui::ScrollArea::vertical().min_scrolled_height(0.).max_height((ui.ctx().content_rect().height()-100.).max(120.)).show(ui,|ui|{
                ui.heading("Navigate and inspect");
                // Compact camera-action rows first; pacing and manipulation
                // detail stays secondary, preserving every existing fence.
                for row in [
                    "Orbit · primary drag",
                    "Pan · Shift-primary, secondary or middle drag",
                    "Zoom · scroll or pinch",
                    "Frame selection and Default view · end Follow",
                    "Escape · closes dialogs and ends gestures",
                ] {
                    ui.label(egui::RichText::new(row).small().color(theme::TEXT));
                }
                ui.add_space(4.);
                ui.label(
                    egui::RichText::new("Select and inspect")
                        .small()
                        .color(theme::MUTED),
                );
                ui.label("Click selects. The Inspector shows the selected object; Outline lists the scene.");
                ui.add_space(4.);
                ui.label(
                    egui::RichText::new("Manipulate").small().color(theme::MUTED),
                );
                ui.label("Applies force while running, or moves eligible free bodies while paused. Focus loss, tool changes, Reset and Stop end the gesture. Release before starting a new one.");
                ui.add_space(4.);
                ui.label(
                    egui::RichText::new("Simulation speed")
                        .small()
                        .color(theme::MUTED),
                );
                ui.label("Realtime follows wall time; Fast advances as quickly as possible without changing the quantum. Start execution to change pacing; Stop and cleanup uncertainty disable changes.");
                let target = ui.checkbox(&mut self.larger_targets,"Larger touch targets");ids.push(target.id);
                });
                // The dismiss action stays outside the scrolling body so it
                // is always fully visible, including with larger targets.
                let close = ui.add(
                    egui::Button::image_and_text(
                        icons::Icon::X.image(theme::TEXT).fit_to_exact_size(egui::vec2(14., 14.)),
                        "Close help",
                    )
                    .fill(egui::Color32::TRANSPARENT),
                );ids.push(close.id);if close.clicked(){self.help_open=false;}
                #[cfg(test)]
                ui.ctx()
                    .data_mut(|d| d.insert_temp(egui::Id::new("help-close-probe"), close.rect));
            });
            #[cfg(test)]
            ui.ctx()
                .data_mut(|d| d.insert_temp(egui::Id::new("modal-bounds"), response.response.rect));
            self.modal_focus.finish(ui.ctx(), &ids);
            if response.should_close() {
                self.help_open = false;
            }
        }
        if self.import_open {
            // The placement surface owns the whole background: a transparent
            // interactable layer absorbs pointer and wheel input over the
            // header, panels, viewport and footer while the sheet stays
            // contextual. Keyboard focus is trapped by modal_focus below.
            let screen = ui.max_rect();
            let backdrop = egui::Area::new(egui::Id::new("import-backdrop"))
                .order(egui::Order::Middle)
                .fixed_pos(screen.min)
                .interactable(true)
                .show(ui.ctx(), |ui| {
                    let (_rect, response) =
                        ui.allocate_exact_size(screen.size(), egui::Sense::click());
                    response.on_hover_cursor(egui::CursorIcon::Default);
                });
            let _ = &backdrop;
            self.modal_focus.begin(ui.ctx(), egui::Id::new("import"));
            let mut ids = Vec::new();
            let import_narrow = canvas.width() < 600.;
            let sheet_width = if import_narrow {
                canvas.width()
            } else {
                (canvas.width() - 24.).min(340.)
            };
            let sheet_pos = if import_narrow {
                // The reserved strip collapses to nothing while the sheet owns
                // it, so anchor directly below the canvas instead of reading
                // the reserved rect back.
                egui::pos2(canvas.left(), canvas.bottom() + theme::STACK_GAP)
            } else {
                egui::pos2(canvas.right() - theme::INSET - sheet_width, panel_top)
            };
            let sheet_max_height = if import_narrow {
                (panel_height - theme::STACK_GAP).max(60.)
            } else {
                (panel_bottom - panel_top).max(120.)
            };
            let sheet_bounds = egui::Rect::from_min_size(
                sheet_pos,
                egui::vec2(sheet_width, sheet_max_height.max(60.)),
            );
            let response = egui::Area::new(egui::Id::new("import-sheet"))
                .fixed_pos(sheet_bounds.min)
                .show(ui.ctx(), |ui| {
                    egui::Resize::default()
                        .fixed_size([sheet_width, sheet_max_height.max(60.)])
                        .show(ui, |ui| {
                            egui::Frame::new()
                                .fill(theme::SURFACE)
                                .inner_margin(18)
                                .stroke(egui::Stroke::new(theme::BORDER_WIDTH, theme::LINE))
                                .corner_radius(theme::RADIUS)
                                .show(ui, |ui| {
                                    ui.set_min_width(sheet_width - 38.);
                                    egui::ScrollArea::vertical()
                                        .min_scrolled_height(0.)
                                        .max_height((sheet_max_height - 36.).max(1.))
                                        .show(ui, |ui| {
                                            self.import_sheet(ui, &mut state, stopped, &mut ids)
                                        });
                                });
                        });
                });
            let sheet_rect = response.response.rect.intersect(sheet_bounds);
            #[cfg(test)]
            ui.ctx()
                .data_mut(|d| d.insert_temp(egui::Id::new("modal-bounds"), sheet_rect));
            self.sheet_bounds = Some(sheet_rect);
            self.captured.push(sheet_rect);
            self.modal_focus.finish(ui.ctx(), &ids);
            let popup_open = egui::Popup::is_any_open(ui.ctx());
            let escape = !popup_open && ui.ctx().input(|i| i.key_pressed(egui::Key::Escape));
            if escape {
                if state.document.cancelable {
                    if let Ok(world) = self.world.lock() {
                        world.cancel_admission();
                    }
                    self.import_cancel_requested = true;
                } else if !state.document.busy {
                    self.edit(Edit::CancelImport);
                    self.import_open = false;
                }
            }
        }
        if !self.import_open && !self.help_open {
            self.modal_focus.restore(ui.ctx());
        }
        if let Some(request) = &state.requests.pending_scene
            && matches!(
                request.action,
                scene::SceneAction::Pick(_) | scene::SceneAction::SelectBody(_)
            )
        {
            self.selection_pending = Some(request.frame.clone());
        }
        state.requests.viewport_size = Some([canvas.width(), canvas.height()]);
        if let Ok(mut shared) = display.lock() {
            shared.request(state.generation, state.requests);
        }
    }
}
impl eframe::App for Desktop {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.application_termination(ctx);
    }
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.draw(ui);
    }
}

impl Desktop {
    // Logic also runs while occluded: termination must not depend on painting.
    fn application_termination(&mut self, ctx: &egui::Context) {
        let (requested, finished, busy, cleanup_failed) = {
            let Ok(state) = self.display.lock() else {
                return;
            };
            (
                state.close_requested,
                state.finished,
                state.document.busy,
                state.cleanup_failed,
            )
        };
        if requested && !self.allow_close {
            self.departure.begin(PendingAction::Close);
            if self
                .departure
                .take_ready(finished && !busy, cleanup_failed)
                .is_some()
            {
                self.allow_close = true;
                eprintln!("phoxal-simulator: application close accepted");
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            ctx.request_repaint();
        }
    }
    /// Right footer group: tabular simulation time and the Console action.
    fn footer_right(&mut self, ui: &mut egui::Ui, state: &RunState) {
        let selected = self.console_open;
        let target = controls::control_size(ui)
            .max(if self.larger_targets { 40. } else { 0. })
            .max(28.);
        let (rect, response) =
            ui.allocate_exact_size(egui::vec2(target + 74., target), egui::Sense::click());
        let fill = if selected {
            theme::SELECTED_FILL
        } else if response.hovered() {
            theme::RAISED_FILL
        } else {
            egui::Color32::TRANSPARENT
        };
        if fill != egui::Color32::TRANSPARENT {
            ui.painter()
                .rect_filled(rect, egui::CornerRadius::same(5), fill);
        }
        let color = if selected {
            theme::ACCENT_TEXT
        } else {
            theme::MUTED
        };
        // Genuine terminal glyph before the label, vertically centered.
        let glyph = egui::Rect::from_center_size(
            rect.left_center() + egui::vec2(15., 0.),
            egui::vec2(icons::GLYPH, icons::GLYPH),
        );
        icons::Icon::Terminal.paint_at(ui, glyph, color, icons::GLYPH);
        let painter = ui.painter();
        painter.text(
            glyph.right_center() + egui::vec2(8., 0.),
            egui::Align2::LEFT_CENTER,
            "Console",
            egui::FontId::proportional(12.),
            color,
        );
        // Keyboard focus paints last so it stays visible over selected
        // and hovered fills.
        if response.has_focus() {
            painter.rect_stroke(
                rect,
                egui::CornerRadius::same(5),
                egui::Stroke::new(2., egui::Color32::WHITE),
                egui::StrokeKind::Inside,
            );
        }
        response.widget_info(|| {
            egui::WidgetInfo::selected(egui::WidgetType::Button, true, selected, "Console")
        });
        #[cfg(test)]
        ui.ctx().data_mut(|d| {
            d.insert_temp(egui::Id::new("console-toggle-probe"), (rect, response.id))
        });
        let keyboard = response.has_focus()
            && ui.input(|i| i.key_pressed(egui::Key::Enter) || i.key_pressed(egui::Key::Space));
        if response.clicked() || keyboard {
            self.console_open = !self.console_open;
        }
        let help = if selected {
            "Hide the console drawer"
        } else {
            "Show bounded, copyable diagnostics"
        };
        response.on_hover_text(help);
        let minutes = (state.time_seconds / 60.).floor() as u64;
        let seconds = (state.time_seconds % 60.).floor() as u64;
        let factor = state.speed.map_or(0., |rate| rate.max(0.));
        // Content-width clock directly after Console, vertically centered
        // by the footer row's centered layout.
        ui.label(
            egui::RichText::new(format!("Sim {minutes}:{seconds:02} · {factor:.2}x"))
                .font(egui::FontId::monospace(11.))
                .color(theme::TEXT),
        );
    }
    fn context_panel(&mut self, ui: &mut egui::Ui, state: &mut RunState, stopped: bool) {
        if self.outline_open {
            panel_heading(ui, "Outline", "Close outline", &mut self.outline_open);
            outline(ui, state);
        }
        if self.inspector_open {
            panel_heading(ui, "Inspector", "Close inspector", &mut self.inspector_open);
            if state.selection.is_none() {
                ui.label(egui::RichText::new("Nothing selected").strong());
                ui.label(
                    egui::RichText::new("Click an object in the scene or choose one in Outline")
                        .small()
                        .color(theme::MUTED),
                );
                ui.add_space(6.);
            } else {
                inspector_header(ui, state);
                inspector_tabs(ui, &mut self.motion_tab);
                self.inspector_content(ui, state, self.motion_tab);
            }
            self.robot_actions(ui, state, stopped);
        }
        if ui.button("Close panel").clicked() {
            self.inspector_open = false;
            self.outline_open = false;
        }
    }
    /// Contextual placement sheet: scene context, chosen build, placement,
    /// a prominent real candidate preview, then Place or Cancel. The
    /// candidate stays separate from the confirmed world until Place.
    fn import_sheet(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut RunState,
        stopped: bool,
        ids: &mut Vec<egui::Id>,
    ) {
        ui.heading("Place prepared robot");
        // Compact scene and build identity first; the complete build path
        // stays available as secondary, selectable detail.
        let scene_name = if state.document.scene_name.is_empty() {
            "Default scene"
        } else {
            &state.document.scene_name
        };
        ui.label(
            egui::RichText::new(format!("Scene · {scene_name}"))
                .small()
                .color(theme::MUTED),
        );
        ui.label(egui::RichText::new("Robot").small().color(theme::MUTED));
        if let Some(build) = &self.import_build {
            ui.horizontal(|ui| {
                icons::Icon::Box.paint_at(
                    ui,
                    egui::Rect::from_center_size(
                        ui.cursor().left_center() + egui::vec2(8., 0.),
                        egui::vec2(icons::GLYPH, icons::GLYPH),
                    ),
                    theme::MUTED,
                    icons::GLYPH,
                );
                ui.add_space(icons::GLYPH + 4.);
                // Identity is not inferred from directory names: the
                // admitted candidate's verified identity is primary under
                // the Robot group; before admission only the generic
                // truthful label exists.
                let identity = state.document.candidate.as_ref().map(|candidate| {
                    candidate
                        .identity
                        .lines()
                        .next()
                        .unwrap_or("Prepared robot")
                        .to_owned()
                });
                ui.label(identity.as_deref().unwrap_or("Prepared robot build"));
            });
            egui::CollapsingHeader::new("Build path")
                .default_open(false)
                .show(ui, |ui| {
                    ui.add(
                        egui::Label::new(build.display().to_string())
                            .selectable(true)
                            .wrap(),
                    );
                });
        }
        if let Some(error) = state
            .error
            .as_ref()
            .or(state.document_error.as_ref())
            .or(self.message.as_ref())
        {
            ui.colored_label(theme::ERROR, &error.primary);
            ui.label(&error.action);
        }
        ui.label(egui::RichText::new("Placement").small().color(theme::MUTED));
        ui.label(
            egui::RichText::new("Attachment")
                .small()
                .color(theme::MUTED),
        );
        // Genuine chevron without the duplicated native arrow, through the
        // same menu/popup owner the sheet already uses.
        let (attachment, _popup) =
            egui::containers::menu::MenuButton::from_button(egui::Button::new((
                self.import_target.clone(),
                icons::Icon::ChevronDown
                    .image(theme::MUTED)
                    .fit_to_exact_size(egui::vec2(14., 14.)),
            )))
            .ui(ui, |ui| {
                for a in &state.document.attachments {
                    if ui
                        .selectable_value(&mut self.import_target, a.0.clone(), &a.0)
                        .changed()
                    {
                        self.import_position = a.1;
                        self.import_quaternion = a.2;
                        self.import_yaw = 0.;
                        self.import_dirty = true;
                        ui.close();
                    }
                }
            });
        ids.push(attachment.id);
        // Each label stays grouped with its field. egui's wrapped layout
        // does not break nested horizontal groups, so at narrow sheet
        // widths the groups stack vertically instead of forcing horizontal
        // overflow; wide sheets keep the single wrapped row.
        let mut axis_group = |ui: &mut egui::Ui, label: &str, value: &mut f64, yaw: bool| {
            ui.horizontal(|ui| {
                ui.label(label);
                let field = if yaw {
                    ui.add(egui::DragValue::new(value))
                } else {
                    ui.add(egui::DragValue::new(value).speed(0.01))
                };
                if field.changed() {
                    self.import_dirty = true;
                }
                ids.push(field.id);
            });
        };
        ui.label(
            egui::RichText::new("Position · attachment-local · m")
                .small()
                .color(theme::MUTED),
        );
        if ui.available_width() < 360. {
            ui.vertical(|ui| {
                for (axis, value) in ["X", "Y", "Z"]
                    .into_iter()
                    .zip(self.import_position.iter_mut())
                {
                    axis_group(ui, axis, value, false);
                }
                axis_group(ui, "Local yaw · degrees", &mut self.import_yaw, true);
            });
        } else {
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing.x = 10.;
                for (axis, value) in ["X", "Y", "Z"]
                    .into_iter()
                    .zip(self.import_position.iter_mut())
                {
                    axis_group(ui, axis, value, false);
                }
                axis_group(ui, "Local yaw · degrees", &mut self.import_yaw, true);
            });
        }
        let default_placement =
            state
                .document
                .attachments
                .first()
                .is_some_and(|(name, position, quaternion)| {
                    *name == self.import_target
                        && *position == self.import_position
                        && *quaternion == self.import_quaternion
                        && self.import_yaw == 0.
                });
        if default_placement {
            ui.label(
                egui::RichText::new("Default placement")
                    .small()
                    .color(theme::MUTED),
            );
        }
        if state.document.attachments.is_empty() {
            ui.label("This scene has no eligible named world-level attachment. Add one explicitly in the authored scene.");
        }
        ui.label(egui::RichText::new("Preview").small().color(theme::MUTED));
        ui.label(
            egui::RichText::new("Place adds the robot; it does not start simulation")
                .small()
                .color(theme::MUTED),
        );
        let preview = ui.add_enabled(
            stopped && !self.import_target.is_empty(),
            egui::Button::new("Preview placement"),
        );
        if preview.enabled() {
            ids.push(preview.id);
        }
        // Edited placement re-previews through the same owner so the world
        // view always reflects the placement being confirmed.
        if self.import_dirty && !state.document.busy && stopped {
            self.import_dirty = false;
            if let Some(build) = self.import_build.clone() {
                let quaternion = yaw_orientation(self.import_quaternion, self.import_yaw);
                self.edit(Edit::Import {
                    build,
                    target: self.import_target.clone(),
                    position: self.import_position,
                    quaternion,
                });
            }
        }
        if preview.clicked()
            && let Some(build) = &self.import_build
        {
            let quaternion = yaw_orientation(self.import_quaternion, self.import_yaw);
            self.edit(Edit::Import {
                build: build.clone(),
                target: self.import_target.clone(),
                position: self.import_position,
                quaternion,
            });
        }
        if let Some(candidate) = &state.document.candidate {
            let identity_line = candidate
                .identity
                .lines()
                .next()
                .unwrap_or("Prepared robot")
                .to_owned();
            ui.label(
                egui::RichText::new(format!("Candidate · {identity_line}"))
                    .small()
                    .color(theme::MUTED),
            );
            egui::CollapsingHeader::new("Candidate identity")
                .default_open(false)
                .show(ui, |ui| {
                    ui.add(
                        egui::Label::new(&candidate.identity)
                            .selectable(true)
                            .wrap(),
                    );
                });
            let quaternion = yaw_orientation(self.import_quaternion, self.import_yaw);
            let current = candidate.build == self.import_build.clone().unwrap_or_default()
                && candidate.target == self.import_target
                && candidate.position == self.import_position
                && candidate.quaternion == quaternion;
            // Truthful preview state derived from the actual candidate
            // comparison and outstanding admission, never an unconditional
            // live claim over a previous placement.
            let status = if state.document.busy {
                "Updating preview…"
            } else if current {
                "The world view shows the live candidate preview; nothing is committed until Place."
            } else {
                "Placement changed · the preview shows the previous placement until it updates. Place stays disabled until the displayed candidate matches."
            };
            ui.label(egui::RichText::new(status).small().color(theme::MUTED));
            // Validated Place is the full-width accent primary action with
            // its genuine glyph after the label. Dark text and glyph belong
            // to the enabled accent surface only; stale or busy refusal
            // keeps the readable disabled semantic color.
            let place_enabled = stopped && current;
            let place_foreground = if place_enabled {
                egui::Color32::from_rgb(17, 24, 27)
            } else {
                theme::DISABLED
            };
            let mut place_spec = egui::Button::new((
                egui::RichText::new("Place robot").color(place_foreground),
                icons::Icon::ArrowDownToLine
                    .image(place_foreground)
                    .fit_to_exact_size(egui::vec2(icons::GLYPH, icons::GLYPH)),
            ))
            .min_size(egui::vec2(ui.available_width(), 30.));
            place_spec = if place_enabled {
                place_spec.fill(theme::ACCENT_TEXT)
            } else {
                place_spec.fill(theme::RAISED_FILL)
            };
            let place = ui.add_enabled(stopped && current, place_spec);
            if place.enabled() {
                ids.push(place.id);
            }
            #[cfg(test)]
            ui.ctx()
                .data_mut(|d| d.insert_temp(egui::Id::new("place-probe"), place.rect));
            if place.clicked() {
                self.edit(Edit::Place);
                self.placement_pending = self.message.is_none();
            }
        }
        let cancel = ui.add_enabled(
            !state.document.busy || state.document.cancelable,
            egui::Button::image_and_text(
                icons::Icon::X
                    .image(theme::MUTED)
                    .fit_to_exact_size(egui::vec2(14., 14.)),
                "Cancel",
            )
            .fill(egui::Color32::TRANSPARENT),
        );
        if cancel.enabled() {
            ids.push(cancel.id);
        }
        #[cfg(test)]
        ui.ctx()
            .data_mut(|d| d.insert_temp(egui::Id::new("cancel-probe"), cancel.rect));
        if cancel.clicked() {
            if state.document.busy {
                if let Ok(world) = self.world.lock() {
                    world.cancel_admission();
                }
                self.import_cancel_requested = true;
            } else {
                self.edit(Edit::CancelImport);
                self.import_open = false;
            }
        }
    }
    fn inspector_content(&mut self, ui: &mut egui::Ui, state: &mut RunState, motion: bool) {
        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Wrap);
        let action_target = theme::CONTROL.max(if self.larger_targets { 40. } else { 0. });
        let mut selected_ancestor = None;
        let mut follow_clicked = false;
        let mut clear_should_send = false;
        if let Some(hit) = &state.selection {
            if let Some(body) = state.bodies.get(hit.body) {
                // Compact intrinsic-height rows: the label leads and the
                // right-aligned value stays on the same line, never
                // consuming the panel's remaining height.
                let row = |ui: &mut egui::Ui, label: &str, value: String| {
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(label).small().color(theme::MUTED));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new(value)
                                        .font(egui::FontId::monospace(12.))
                                        .color(theme::TEXT),
                                )
                                .selectable(true),
                            );
                        });
                    });
                };
                let component_rows = |ui: &mut egui::Ui, names: &[&str], values: &[f64]| {
                    for (name, value) in names.iter().zip(values.iter()) {
                        row(ui, name, format!("{value:.3}"));
                    }
                };
                if motion && let Some(view) = state.requests.presented.as_ref() {
                    let snapshot = &view.snapshot;
                    if let (Some(pos), Some(quat), Some(vel)) = (
                        snapshot.body_positions().get(body.id),
                        snapshot.body_orientations().get(body.id),
                        snapshot.body_velocities().get(body.id),
                    ) {
                        ui.label(
                            egui::RichText::new("Position · world frame · body origin · m")
                                .small()
                                .color(theme::MUTED),
                        );
                        component_rows(ui, &["X", "Y", "Z"], pos);
                        ui.add_space(4.);
                        ui.label(
                            egui::RichText::new("Orientation · quaternion W/X/Y/Z")
                                .small()
                                .color(theme::MUTED),
                        );
                        component_rows(ui, &["W", "X", "Y", "Z"], quat);
                        ui.add_space(4.);
                        ui.label(
                            egui::RichText::new("Linear velocity · m/s")
                                .small()
                                .color(theme::MUTED),
                        );
                        component_rows(ui, &["X", "Y", "Z"], &vel[3..]);
                        ui.add_space(4.);
                        ui.label(
                            egui::RichText::new("Angular velocity · rad/s")
                                .small()
                                .color(theme::MUTED),
                        );
                        component_rows(ui, &["X", "Y", "Z"], &vel[..3]);
                    }
                } else if let Some(view) = state.requests.presented.as_ref()
                    && let Some(pos) = view.snapshot.body_positions().get(body.id)
                {
                    ui.label(
                        egui::RichText::new("Position · world frame · body origin · m")
                            .small()
                            .color(theme::MUTED),
                    );
                    component_rows(ui, &["X", "Y", "Z"], pos);
                }
                ui.add_space(6.);

                // Camera scope: Follow tracks the camera only, with the
                // actual bound target independent of the current selection.
                ui.label(egui::RichText::new("Camera").small().color(theme::MUTED));
                let target = follow::candidate(state);
                let following = self.follow.as_ref();
                // Follow moves the camera only; the explanation stays
                // visible, not hidden in a tooltip.
                ui.label(
                    egui::RichText::new("Moves the camera only")
                        .small()
                        .color(theme::MUTED),
                );
                if let Some(active) = following {
                    ui.label(format!("Following {}", active.label));
                    if ui
                        .add(panel_button(
                            action_target,
                            "Stop following",
                            Some(icons::Icon::Crosshair),
                        ))
                        .clicked()
                    {
                        follow_clicked = true;
                    }
                } else if let Some((_, label)) = &target {
                    // Target identity stays visible; binding additionally
                    // needs the presented view the owner binds against.
                    ui.label(format!("Target: {label}"));
                    let robot_part = state.robot_root.is_some_and(|root| {
                        state.bodies.get(body.id).is_some_and(|b| {
                            b.id == root || follow::is_descendant(&state.bodies, b.id, root)
                        })
                    });
                    let action = if robot_part {
                        "Follow robot"
                    } else {
                        "Follow body"
                    };
                    if state.requests.presented.is_some() {
                        let follow_action = ui.add(panel_button(
                            action_target,
                            action,
                            Some(icons::Icon::Crosshair),
                        ));
                        #[cfg(test)]
                        ui.ctx().data_mut(|d| {
                            d.insert_temp(
                                egui::Id::new("follow-action-id-probe"),
                                (
                                    follow_action.rect,
                                    follow_action.id,
                                    follow_action.enabled(),
                                ),
                            );
                        });
                        if follow_action.clicked() {
                            follow_clicked = true;
                        }
                    } else {
                        ui.add_enabled(
                            false,
                            panel_button(action_target, action, Some(icons::Icon::Crosshair)),
                        )
                        .on_disabled_hover_text("Wait for the scene presentation to bind a target");
                        ui.label(
                            egui::RichText::new(
                                "Waiting for the scene presentation to bind a target",
                            )
                            .small()
                            .color(theme::MUTED),
                        );
                    }
                } else if state.requests.presented.is_none() {
                    ui.weak("Waiting for the scene presentation to bind a target");
                } else {
                    ui.weak("Select a robot part or movable body to follow");
                }
                ui.add_space(6.);

                // Selection scope.
                ui.label(egui::RichText::new("Selection").small().color(theme::MUTED));
                let clear_action = ui.add(panel_button(action_target, "Clear selection", None));
                #[cfg(test)]
                ui.ctx().data_mut(|d| {
                    d.insert_temp(
                        egui::Id::new("clear-action-id-probe"),
                        (clear_action.rect, clear_action.id),
                    );
                });
                clear_should_send = clear_action.clicked();
                ui.label(
                    egui::RichText::new("Clears inspection; keeps the object in the scene")
                        .small()
                        .color(theme::MUTED),
                );
                if let Some(ancestor) = free_ancestor(&state.bodies, body.id) {
                    // The same friendly identity the Inspector header uses;
                    // the movement scope is named instead of internal jargon.
                    // Robot wording requires verified membership of the
                    // robot root; a scene-only movable ancestor is labeled
                    // by what it is, never invented robot identity.
                    let ancestor_body = &state.bodies[ancestor];
                    let robot_member = state
                        .robot_root
                        .is_some_and(|root| follow::is_descendant(&state.bodies, ancestor, root));
                    let action = if robot_member {
                        "Select robot body"
                    } else {
                        "Select movable body"
                    };
                    let name = friendly_body_name(state, ancestor_body);
                    let select = ui.add(panel_button(
                        action_target,
                        format!("{action} · {name}"),
                        None,
                    ));
                    #[cfg(test)]
                    ui.ctx().data_mut(|d| {
                        d.insert_temp(egui::Id::new("ancestor-action-probe"), select.rect)
                    });
                    if select.clicked() {
                        selected_ancestor = Some(ancestor);
                    }
                    ui.label(
                        egui::RichText::new("Moves and forces apply to that body")
                            .small()
                            .color(theme::MUTED),
                    );
                }
                ui.add_space(6.);

                egui::CollapsingHeader::new("Diagnostics")
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(format!(
                                "Native body {} · {}",
                                body.id,
                                body.mobility.label()
                            ))
                            .small()
                            .color(theme::MUTED),
                        )
                        .on_hover_text(match body.mobility {
                            BodyMobility::Fixed => "Fixed bodies are selectable but cannot be manipulated.",
                            BodyMobility::FreeJoint => "Running force, or paused translation when this free body has no active weld/connect constraint.",
                            BodyMobility::Articulated => "Running physical force only; paused pose edits require the selected body's own free joint.",
                        });
                        ui.label(
                            egui::RichText::new(format!(
                                "Geom {}",
                                hit.geom
                                    .map(|id| id.to_string())
                                    .unwrap_or_else(|| "none".into())
                            ))
                            .small()
                            .color(theme::MUTED),
                        );
                        ui.add(
                            egui::Label::new(
                                egui::RichText::new(format!(
                                    "Pick hit [{:.3}, {:.3}, {:.3}] m",
                                    hit.point[0], hit.point[1], hit.point[2]
                                ))
                                .font(egui::FontId::monospace(11.)),
                            )
                            .selectable(true),
                        );
                        if let Some(view) = state.requests.presented.clone() {
                            let identity = view.identity.epoch.model.to_hex();
                            ui.small(format!("Model {}", &identity[..12]))
                                .on_hover_text(identity);
                        } else {
                            ui.small("Model unavailable");
                        }
                    });
            }
            if clear_should_send {
                queue_scene(state, scene::SceneAction::Clear);
            }
            if let Some(ancestor) = selected_ancestor {
                queue_scene(state, scene::SceneAction::SelectBody(ancestor));
            }
            if follow_clicked {
                let target = follow::candidate(state);
                self.follow = if self.follow.is_some() {
                    None
                } else {
                    target.and_then(|(body, label)| {
                        state
                            .requests
                            .presented
                            .as_ref()
                            .map(|view| follow::Target {
                                epoch: view.identity.epoch.clone(),
                                body,
                                label,
                            })
                    })
                };
            }
        }
    }

    fn robot_actions(&mut self, ui: &mut egui::Ui, state: &mut RunState, stopped: bool) {
        if state.document.robot.is_none() && state.document.removed_robot.is_none() {
            return;
        }
        // Robot in scene is its own scope, independent of the current
        // selection: it concerns the document robot as a whole.
        ui.separator();
        ui.label(
            egui::RichText::new(if state.document.robot.is_some() {
                "Robot in scene"
            } else {
                "Removed robot"
            })
            .small()
            .color(theme::MUTED),
        );
        let action_target = theme::CONTROL.max(if self.larger_targets { 40. } else { 0. });
        // The authoritative restriction, most severe first: cleanup
        // uncertainty dominates, then outstanding edits, then an active
        // simulation. Each states its real recovery, never an impossible
        // generic instruction.
        enum Restriction {
            Cleanup,
            Busy,
            Running,
            None,
        }
        let restriction = if state.cleanup_failed || state.stopping {
            Restriction::Cleanup
        } else if state.document.busy {
            Restriction::Busy
        } else if !stopped {
            Restriction::Running
        } else {
            Restriction::None
        };
        if let Some(robot) = &state.document.robot {
            ui.label(robot);
            let can_remove = matches!(restriction, Restriction::None);
            let remove_action = ui.add_enabled(
                can_remove,
                panel_button(action_target, "Remove from scene", None),
            );
            #[cfg(test)]
            ui.ctx().data_mut(|d| {
                d.insert_temp(
                    egui::Id::new("remove-action-id-probe"),
                    (
                        remove_action.rect,
                        remove_action.id,
                        remove_action.enabled(),
                    ),
                );
            });
            if remove_action.clicked() {
                self.edit(Edit::Remove);
            }
            let reason = match restriction {
                Restriction::Cleanup => {
                    "Cleanup incomplete. Resolve the shutdown failure and confirm cleanup before editing the robot"
                }
                Restriction::Busy => "Updating simulation…",
                Restriction::Running => "Stop the simulation before removing the robot",
                Restriction::None => "Removes the whole robot; source files are kept",
            };
            ui.label(egui::RichText::new(reason).small().color(theme::MUTED));
        }
        // Conditional Undo exists only while an actual removal checkpoint
        // is available; its availability follows the same authoritative
        // restriction predicates.
        if let Some(robot) = &state.document.removed_robot {
            ui.label(robot);
            let can_undo = matches!(restriction, Restriction::None);
            let undo_action =
                ui.add_enabled(can_undo, panel_button(action_target, "Undo removal", None));
            #[cfg(test)]
            ui.ctx().data_mut(|d| {
                d.insert_temp(
                    egui::Id::new("undo-action-id-probe"),
                    (undo_action.rect, undo_action.id, undo_action.enabled()),
                );
            });
            if undo_action.clicked() {
                self.edit(Edit::Undo);
            }
            let undo_reason = match restriction {
                Restriction::Cleanup => {
                    "Cleanup incomplete. Resolve the shutdown failure and confirm cleanup before undoing".to_owned()
                }
                Restriction::Busy => "Updating simulation…".to_owned(),
                Restriction::Running => "Stop the simulation before undoing the removal".to_owned(),
                Restriction::None => format!("Restores {robot} at its original placement"),
            };
            ui.label(egui::RichText::new(undo_reason).small().color(theme::MUTED));
        }
    }
    fn viewport(&mut self, ui: &mut egui::Ui, state: &mut RunState) {
        if let Some(texture) = &self.texture {
            let pickable = state.ready
                && state
                    .requests
                    .presented
                    .as_ref()
                    .is_some_and(|view| Some(view.camera) == self.view_camera);
            let camera = state.ready.then_some(self.view_camera).flatten();
            let blocked = self.import_open || self.help_open;
            let input = viewport::show(
                ui,
                texture,
                if blocked { None } else { camera },
                pickable && !blocked,
                self.manipulate && !blocked,
                &self.captured,
                state.requests.presented.as_ref().map_or(0, |view| {
                    egui::Id::new((
                        &view.identity.epoch.execution,
                        view.identity.epoch.model.to_hex(),
                        view.identity.epoch.generation,
                        state.running,
                    ))
                    .value()
                }),
            );
            // While placement is open, the world region presents the real
            // rendered candidate at its own aspect, clearly labeled and
            // separate from the confirmed world that stays visible around
            // it. Nothing is committed until Place.
            if self.import_open
                && let Some(candidate) = state.document.candidate.clone()
            {
                let key = format!(
                    "{}|{:?}|{:?}",
                    candidate.identity, candidate.position, candidate.quaternion
                );
                if self
                    .candidate_texture
                    .as_ref()
                    .is_none_or(|(held, _)| *held != key)
                {
                    let image = egui::ColorImage::from_rgb(
                        candidate.image.resolution(),
                        candidate.image.rgb(),
                    );
                    let handle = ui.ctx().load_texture(
                        "candidate-preview",
                        image,
                        egui::TextureOptions::LINEAR,
                    );
                    self.candidate_texture = Some((key, handle));
                }
                if let Some((_, handle)) = &self.candidate_texture {
                    // Fit into the measured unobscured world region: the
                    // placement sheet's own bounds are excluded so the whole
                    // preview plus its label stays reviewable beside the
                    // sheet, at the candidate's own aspect.
                    let canvas = ui.max_rect();
                    let sheet = self.sheet_bounds.unwrap_or(egui::Rect::from_min_max(
                        egui::pos2(canvas.right() - 356., canvas.top()),
                        egui::pos2(canvas.right(), canvas.bottom()),
                    ));
                    let mut region = canvas;
                    // The committed-world status badge paints in its own
                    // measured bounds; the preview never covers it.
                    if let Some(status) = self.status_bounds
                        && status.intersects(canvas)
                    {
                        region.min.y = region.min.y.max(status.bottom() + theme::STACK_GAP);
                    }
                    if sheet.intersects(canvas) {
                        if sheet.min.x > canvas.center().x {
                            region.max.x = sheet.min.x - theme::INSET;
                        } else if sheet.max.x < canvas.center().x {
                            region.min.x = sheet.max.x + theme::INSET;
                        }
                        if sheet.min.y > canvas.center().y {
                            region.max.y = sheet.min.y - theme::STACK_GAP;
                        } else if sheet.max.y < canvas.center().y {
                            region.min.y = sheet.max.y + theme::STACK_GAP;
                        }
                    }
                    region.max.x = region.max.x.min(canvas.max.x);
                    region.max.y = region.max.y.min(canvas.max.y);
                    region.min.x = region.min.x.max(canvas.min.x);
                    region.min.y = region.min.y.max(canvas.min.y);
                    let size_vec = handle.size_vec2();
                    let aspect = size_vec.x / size_vec.y;
                    let painter = egui::Painter::new(
                        ui.ctx().clone(),
                        egui::LayerId::new(
                            egui::Order::Foreground,
                            egui::Id::new("candidate-preview"),
                        ),
                        ui.clip_rect(),
                    );
                    // Measure the actual label inside the available frame
                    // width (it wraps if needed), reserve its real top band
                    // plus a gap, and fit the aspect-preserved image into
                    // the space below it. The frame is then the union, so
                    // text and image never overlap and everything stays
                    // inside the measured unobscured region at any width
                    // or text scale.
                    let pad = 8.;
                    let avail = egui::Vec2::new(region.width() * 0.95, region.height() * 0.95);
                    let galley = painter.layout(
                        "Candidate preview · not placed".to_owned(),
                        egui::TextStyle::Body.resolve(ui.style()),
                        theme::TEXT,
                        (avail.x - 2. * pad).max(20.),
                    );
                    let label_size = galley.size();
                    let band = pad + label_size.y + pad;
                    let max_image = egui::Vec2::new(
                        (avail.x - 2. * pad).max(1.),
                        (avail.y - band - pad).max(1.),
                    );
                    let mut size = egui::vec2(max_image.y * aspect, max_image.y);
                    if size.x > max_image.x {
                        size = egui::vec2(max_image.x, max_image.x / aspect);
                    }
                    let frame =
                        egui::vec2(size.x.max(label_size.x) + 2. * pad, band + size.y + pad);
                    let center = region.center();
                    let frame_rect = egui::Rect::from_center_size(center, frame);
                    painter.rect_filled(
                        frame_rect,
                        theme::RADIUS,
                        egui::Color32::from_black_alpha(210),
                    );
                    painter.rect_stroke(
                        frame_rect,
                        theme::RADIUS,
                        egui::Stroke::new(theme::BORDER_WIDTH, theme::ACCENT_TEXT),
                        egui::StrokeKind::Inside,
                    );
                    let label_pos = frame_rect.left_top() + egui::vec2(pad, pad);
                    let image_min = egui::pos2(
                        frame_rect.left() + (frame.x - size.x) * 0.5,
                        frame_rect.top() + band,
                    );
                    let image_rect = egui::Rect::from_min_size(image_min, size);
                    // Full normalized UVs: the preview samples the entire
                    // rendered candidate, never a single corner texel.
                    painter.image(
                        handle.id(),
                        image_rect,
                        egui::Rect::from_min_max(egui::pos2(0., 0.), egui::pos2(1., 1.)),
                        egui::Color32::WHITE,
                    );
                    painter.galley(label_pos, galley, theme::TEXT);
                }
            }
            if let Some(cut) = input.pointer_cut {
                self.pointer_cut = Some(cut);
            }
            #[cfg(test)]
            {
                self.last_viewport = input.canvas;
            }
            if let (Some(camera), Some(view)) = (input.camera, state.requests.presented.as_ref()) {
                if input.pan {
                    self.follow = None;
                }
                self.view_camera = Some(camera);
                state.requests.pending_camera = Some(CameraRequest {
                    epoch: view.identity.epoch.clone(),
                    camera,
                });
            }
            if input.cancel || !state.ready || state.finished {
                if let Some((id, _, _, _)) = self.gesture.take() {
                    state.requests.drag_mailbox.end(id);
                }
            } else {
                if let (Some((start, xy, height)), Some(view)) =
                    (input.begin, state.requests.presented.as_ref())
                {
                    if let Some((id, _, _, _)) = self.gesture.take() {
                        state.requests.drag_mailbox.end(id);
                    }
                    self.next_gesture = self.next_gesture.saturating_add(1);
                    let id = self.next_gesture;
                    self.gesture = Some((id, view.identity.epoch.clone(), start, height));
                    state.requests.drag_mailbox.begin = Some(scene::DragBegin {
                        id,
                        frame: view.identity.clone(),
                        xy,
                        received: std::time::Instant::now(),
                    });
                }
                if let (Some((id, epoch, start, height)), Some((position, _))) =
                    (&self.gesture, input.held)
                {
                    state.requests.drag_mailbox.update = Some(scene::DragUpdate {
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
                    } else {
                        ui.spinner();
                        ui.label("Preparing native scene…");
                    }
                },
            );
        }
    }
}
fn notice(ui: &mut egui::Ui, state: &RunState, message: Option<&Notice>) {
    let error = state
        .error
        .as_ref()
        .or(state.document_error.as_ref())
        .or(message);
    if let Some(error) = error {
        // Semantic emphasis: hard errors read as errors, informational
        // notices as warnings, on the dark notice surface.
        let severity = if state.error.is_some() {
            theme::ERROR
        } else {
            theme::WARNING
        };
        ui.add(
            egui::Label::new(egui::RichText::new(&error.primary).strong().color(severity)).wrap(),
        );
    }
    if state.cleanup_failed {
        ui.label("Cleanup incomplete. Restart is disabled.");
    }
    if let Some(error) = error {
        ui.label(if state.cleanup_failed {
            "Next: close this window, resolve the shutdown failure in Error details, and confirm the execution's participants have stopped and remote authority/session cleanup is complete before reopening the simulator.".to_owned()
        } else {format!("Next: {}",error.action)});
    }
    if let Some(error) = &state.interaction_error {
        ui.label(error);
    }
    notice_details(ui, state, message);
}

fn notice_details(ui: &mut egui::Ui, state: &RunState, message: Option<&Notice>) {
    if let Some(error) = state
        .error
        .as_ref()
        .or(state.document_error.as_ref())
        .or(message)
    {
        // Reserve error wording for execution failures; scoped admission
        // refusals and informational notices expose plain Details.
        let is_error = state.error.is_some();
        ui.collapsing(if is_error { "Error details" } else { "Details" }, |ui| {
            if ui.button("Copy details").clicked() {
                ui.ctx().copy_text(error.details.clone());
            }
            egui::ScrollArea::vertical()
                .min_scrolled_height(0.)
                .max_height(120.0)
                .show(ui, |ui| {
                    ui.add(egui::Label::new(&error.details).selectable(true));
                });
        });
    }
}

/// Scoped secondary action treatment for panel buttons: a visible raised
/// resting surface with normal 30pt / larger 40pt targets, distinct from
/// the deliberately quiet global toolbars.
fn panel_button(
    target: f32,
    label: impl Into<egui::WidgetText>,
    icon: Option<icons::Icon>,
) -> egui::Button<'static> {
    let text = label.into();
    let base = egui::vec2(target, target);
    match icon {
        Some(icon) => egui::Button::image_and_text(
            icon.image(theme::MUTED)
                .fit_to_exact_size(egui::vec2(icons::GLYPH, icons::GLYPH)),
            text,
        )
        .fill(theme::RAISED_FILL)
        .min_size(base),
        None => egui::Button::new(text)
            .fill(theme::RAISED_FILL)
            .min_size(base),
    }
}

fn queue_scene(state: &mut RunState, action: scene::SceneAction) {
    if let Some(view) = state.requests.presented.clone() {
        state.scene_result = None;
        state.requests.pending_scene = Some(SceneRequest {
            frame: view.identity.clone(),
            action,
        });
    }
}
fn diagnostics(
    ui: &mut egui::Ui,
    state: &RunState,
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
            .min_scrolled_height(0.)
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

/// Compact panel heading with a text-action close control.
fn panel_heading(ui: &mut egui::Ui, title: &str, close_name: &str, open: &mut bool) {
    ui.horizontal(|ui| {
        ui.strong(egui::RichText::new(title).small());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let close = ui.add_enabled(
                true,
                egui::Button::new(
                    icons::Icon::X
                        .image(theme::MUTED)
                        .fit_to_exact_size(egui::vec2(14., 14.)),
                )
                .fill(egui::Color32::TRANSPARENT)
                .min_size(egui::vec2(22., 22.)),
            );
            #[cfg(test)]
            ui.ctx().data_mut(|d| {
                let id = egui::Id::new("control-probes");
                let mut values = d
                    .get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(id)
                    .unwrap_or_default();
                values.push((
                    close_name.to_owned(),
                    close.rect,
                    close.id,
                    false,
                    false,
                    close.enabled(),
                ));
                d.insert_temp(id, values);
            });
            close.widget_info(|| {
                egui::WidgetInfo::labeled(egui::WidgetType::Button, true, close_name)
            });
            if close.on_hover_text(close_name).clicked() {
                *open = false;
            }
        });
    });
}
/// Inspector tabs use a teal underline for selection, not boxed buttons.
fn inspector_tabs(ui: &mut egui::Ui, motion: &mut bool) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 14.;
        for (label, active) in [("Properties", !*motion), ("Motion", *motion)] {
            let text = egui::RichText::new(label).color(if active {
                theme::ACCENT_TEXT
            } else {
                theme::MUTED
            });
            let text = if active { text.underline() } else { text };
            // Quiet real button: resting cells stay borderless through the
            // theme's transparent inactive fill, while keyboard focus gets
            // the theme's distinct white active outline (never Stroke::NONE,
            // which would erase the focused outline too). Selection remains
            // the teal underline.
            let response = ui.add(egui::Button::new(text));
            #[cfg(test)]
            ui.ctx().data_mut(|d| {
                let id = egui::Id::new("tab-probes");
                let mut tabs = d
                    .get_temp::<Vec<(&'static str, egui::Rect, egui::Id)>>(id)
                    .unwrap_or_default();
                tabs.push((label, response.rect, response.id));
                d.insert_temp(id, tabs);
            });
            response
                .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, label));
            if response.clicked() {
                *motion = label == "Motion";
            }
        }
    });
    ui.painter().hline(
        ui.max_rect().x_range(),
        ui.cursor().top() + 2.,
        egui::Stroke::new(theme::BORDER_WIDTH, theme::LINE),
    );
    ui.add_space(6.);
}
fn outline(ui: &mut egui::Ui, state: &mut RunState) {
    fn branch(ui: &mut egui::Ui, state: &mut RunState, id: usize, depth: usize) {
        let Some(body) = state.bodies.get(id).cloned() else {
            return;
        };
        if depth > state.bodies.len() {
            return;
        }
        let prefix = state
            .document
            .robot
            .as_ref()
            .map(|name| format!("{name}__"));
        let member = state
            .robot_root
            .is_some_and(|root| follow::is_descendant(&state.bodies, id, root));
        let short = if member {
            prefix
                .as_ref()
                .and_then(|p| body.name.strip_prefix(p))
                .unwrap_or(&body.name)
        } else {
            &body.name
        };
        let label = if id == 0 {
            "World".to_owned()
        } else if short.is_empty() {
            format!("Body {id}")
        } else if state.robot_root == Some(id)
            && let Some(robot) = &state.document.robot
        {
            // Verified robot membership at its root body, without a new
            // wrapper node or name-based inference.
            format!("{short} · {robot}")
        } else {
            short.to_owned()
        };
        let children: Vec<_> = state
            .bodies
            .iter()
            .filter(|b| b.parent == Some(id) && b.id != id)
            .map(|b| b.id)
            .collect();
        let mut node = egui::collapsing_header::CollapsingState::load_with_default_open(
            ui.ctx(),
            ui.id().with((
                state
                    .requests
                    .presented
                    .as_ref()
                    .map(|v| v.identity.epoch.model.to_hex()),
                id,
            )),
            id == 0,
        );
        // Selection and expansion are separate actions: the chevron is its
        // own toggle and the label (pointer and keyboard) selects the body,
        // so activating a row never needs to guess which was intended.
        // The arrow strip is reserved first but painted only after the row
        // exists, so the glyph centers on the real row; children are shown
        // indented under their parent header.
        let header = ui.horizontal(|ui| {
            let strip_rect = ui
                .allocate_exact_size(egui::vec2(20., 1.), egui::Sense::hover())
                .0;
            // The World root carries its genuine globe entity marker; other
            // rows stay text-only, matching the approved outline.
            let marker = if id == 0 { 18. } else { 0. };
            if marker > 0. {
                ui.add_space(marker);
            }
            let response = ui
                .selectable_label(
                    state.selection.as_ref().is_some_and(|s| s.body == id),
                    label,
                )
                .on_hover_text(format!("{} · Native body {id}", body.name));
            let arrow_rect = egui::Rect::from_center_size(
                egui::pos2(strip_rect.left() + 10., response.rect.center().y),
                egui::vec2(14., 14.),
            );
            let open = node.is_open();
            let (glyph, rotated) = if open {
                (icons::Icon::ChevronDown, false)
            } else {
                // The genuine chevron asset rotated to point right.
                (icons::Icon::ChevronDown, true)
            };
            glyph.paint_at_rotated(ui, arrow_rect, theme::MUTED, 14., rotated);
            if id == 0 {
                let globe = egui::Rect::from_center_size(
                    egui::pos2(strip_rect.right() + 9., response.rect.center().y),
                    egui::vec2(14., 14.),
                );
                icons::Icon::Globe2.paint_at(ui, globe, theme::MUTED, 14.);
            }
            #[cfg(test)]
            ui.ctx().data_mut(|d| {
                let key = egui::Id::new("outline-arrow-probes");
                let mut rows = d
                    .get_temp::<Vec<(usize, egui::Rect)>>(key)
                    .unwrap_or_default();
                rows.push((id, arrow_rect));
                d.insert_temp(key, rows);
                let row_key = egui::Id::new("outline-row-probes");
                let mut row_probes = d
                    .get_temp::<Vec<(usize, egui::Rect)>>(row_key)
                    .unwrap_or_default();
                row_probes.push((id, response.rect));
                d.insert_temp(row_key, row_probes);
                let ids_key = egui::Id::new("outline-row-id-probes");
                let mut ids = d
                    .get_temp::<Vec<(usize, egui::Id)>>(ids_key)
                    .unwrap_or_default();
                ids.push((id, response.id));
                d.insert_temp(ids_key, ids);
            });
            let arrow = ui.interact(
                arrow_rect,
                ui.id().with(("outline-arrow", id)),
                egui::Sense::click(),
            );
            if arrow.clicked() {
                node.toggle(ui);
                node.store(ui.ctx());
            }
            if response.clicked() {
                queue_scene(state, scene::SceneAction::SelectBody(id));
            }
            response
        });
        node.show_body_indented(&header.response, ui, |ui| {
            for child in children {
                branch(ui, state, child, depth + 1);
            }
        });
    }
    if state.bodies.is_empty() {
        ui.weak("No scene objects available");
    } else {
        branch(ui, state, 0, 0);
    }
}
fn friendly_body_name(state: &RunState, body: &crate::mujoco::NativeBody) -> String {
    let member = state
        .robot_root
        .is_some_and(|root| follow::is_descendant(&state.bodies, body.id, root));
    if member
        && let Some(name) = &state.document.robot
        && let Some(short) = body.name.strip_prefix(&format!("{name}__"))
    {
        return short.to_owned();
    }
    if body.name.is_empty() {
        format!("Body {}", body.id)
    } else {
        body.name.clone()
    }
}

/// The selected object leads with its friendly name and truthful type,
/// before the Properties/Motion tabs.
fn inspector_header(ui: &mut egui::Ui, state: &RunState) {
    if let Some(hit) = &state.selection
        && let Some(body) = state.bodies.get(hit.body)
    {
        ui.heading(egui::RichText::new(friendly_body_name(state, body)).strong());
        // Verified relationship only: robot membership comes from the
        // robot root and document identity, never the object's name.
        let member = state
            .robot_root
            .is_some_and(|root| follow::is_descendant(&state.bodies, body.id, root));
        let qualifier = if member && let Some(name) = &state.document.robot {
            format!("Part of {name}")
        } else {
            "Scene body".to_owned()
        };
        ui.label(egui::RichText::new(qualifier).small().color(theme::MUTED));
    }
}

/// The wide Inspector and narrow context panel share the same protected
/// scrolling gutter. Reserve the fully expanded thumb width plus a small
/// gap, even while the floating scrollbar is thin or animating.
fn context_scroll(ui: &mut egui::Ui, height: f32, contents: impl FnOnce(&mut egui::Ui)) {
    ui.scope(|ui| {
        let scroll = &mut ui.spacing_mut().scroll;
        scroll.floating_allocated_width = scroll.bar_width + 4.;
        egui::ScrollArea::vertical()
            .min_scrolled_height(0.)
            .max_height(height)
            .show(ui, contents);
    });
}
