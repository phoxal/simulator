use crate::mujoco::ViewCamera;
use eframe::egui;

#[derive(Clone, Copy, Debug)]
pub(super) struct PointerCut {
    pub pressed: bool,
    pub released: bool,
    pub held: bool,
    pub started: bool,
    pub stopped: bool,
}

#[derive(Default)]
pub(super) struct Input {
    pub pointer_cut: Option<PointerCut>,
    pub camera: Option<ViewCamera>,
    pub pick: Option<[f64; 2]>,
    pub begin: Option<(egui::Pos2, [f64; 2], f32)>,
    pub held: Option<(egui::Pos2, f32)>,
    pub cancel: bool,
    #[cfg(test)]
    pub canvas: Option<egui::Rect>,
}

/// Fractions in MuJoCo's left/bottom origin, independent of DPI and letterboxing.
pub(super) fn coordinates(image: egui::Rect, pointer: egui::Pos2) -> Option<[f64; 2]> {
    if image.width() <= 0.0 || image.height() <= 0.0 || !image.contains(pointer) {
        return None;
    }
    Some([
        f64::from((pointer.x - image.left()) / image.width()),
        f64::from((image.bottom() - pointer.y) / image.height()),
    ])
}
pub(crate) fn valid_camera(view: ViewCamera) -> bool {
    view.look_at.iter().all(|v| v.is_finite() && v.abs() <= 1e6)
        && view.distance.is_finite()
        && (0.01..=100_000.0).contains(&view.distance)
        && view.azimuth.is_finite()
        && view.elevation.is_finite()
        && (-89.0..=89.0).contains(&view.elevation)
}
#[derive(Clone, Copy, Debug, PartialEq)]
enum Gesture {
    Orbit,
    Pan,
}
fn gesture(secondary: bool, middle: bool, shift: bool) -> Option<Gesture> {
    if middle || (secondary && shift) {
        Some(Gesture::Pan)
    } else if secondary {
        Some(Gesture::Orbit)
    } else {
        None
    }
}
fn transform(mut view: ViewCamera, gesture: Gesture, delta: egui::Vec2, height: f32) -> ViewCamera {
    if !delta.is_finite() || !height.is_finite() || height <= 0.0 {
        return view;
    }
    match gesture {
        Gesture::Orbit => {
            view.azimuth = (view.azimuth - f64::from(delta.x) * 0.4).rem_euclid(360.0);
            view.elevation = (view.elevation + f64::from(delta.y) * 0.4).clamp(-89.0, 89.0);
        }
        Gesture::Pan => {
            let az = view.azimuth.to_radians();
            let el = view.elevation.to_radians();
            let right = [az.sin(), -az.cos(), 0.0];
            let up = [-el.sin() * az.cos(), -el.sin() * az.sin(), el.cos()];
            let scale = view.distance / f64::from(height);
            for axis in 0..3 {
                view.look_at[axis] = (view.look_at[axis]
                    - f64::from(delta.x) * scale * right[axis]
                    + f64::from(delta.y) * scale * up[axis])
                    .clamp(-1e6, 1e6);
            }
        }
    }
    view
}
fn zoom(mut view: ViewCamera, scroll: f32, pinch: f32) -> ViewCamera {
    if !scroll.is_finite() || !pinch.is_finite() || pinch <= 0.0 {
        return view;
    }
    let factor = if pinch != 1.0 {
        1.0 / f64::from(pinch)
    } else {
        (-f64::from(scroll) * 0.003).exp()
    };
    view.distance = (view.distance * factor).clamp(0.01, 100_000.0);
    view
}

pub(super) fn drag_displacement(
    start: egui::Pos2,
    pointer: egui::Pos2,
    pinned_height: f32,
) -> [f64; 2] {
    if !pinned_height.is_finite()
        || pinned_height <= 0.0
        || !start.is_finite()
        || !pointer.is_finite()
    {
        return [f64::NAN; 2];
    }
    [
        ((f64::from(pointer.x) - f64::from(start.x)) / f64::from(pinned_height)).clamp(-2.0, 2.0),
        ((f64::from(start.y) - f64::from(pointer.y)) / f64::from(pinned_height)).clamp(-2.0, 2.0),
    ]
}
fn canceled(held: bool, focused: bool, escape: bool, enabled: bool) -> bool {
    !held || !focused || escape || !enabled
}

pub(super) fn show(
    ui: &mut egui::Ui,
    texture: &egui::TextureHandle,
    camera: Option<ViewCamera>,
    can_pick: bool,
) -> Input {
    let available = ui.available_size().max(egui::vec2(1.0, 1.0));
    let dimensions = texture.size_vec2();
    let scale = (available.x / dimensions.x).min(available.y / dimensions.y);
    let (canvas, _) = ui.allocate_exact_size(available, egui::Sense::hover());
    ui.painter()
        .rect_filled(canvas, 8.0, egui::Color32::from_rgb(11, 14, 19));
    let image = egui::Rect::from_center_size(canvas.center(), dimensions * scale);
    ui.painter().image(
        texture.id(),
        image,
        egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
        egui::Color32::WHITE,
    );
    let response = ui.interact(
        image,
        ui.id().with("native_view"),
        egui::Sense::click_and_drag(),
    );
    let mut output = Input::default();
    #[cfg(test)]
    {
        output.canvas = Some(canvas);
    }
    if let Some(mut view) = camera {
        let before = view;
        let (delta, shift, scroll, pinch) = ui.input(|i| {
            (
                i.pointer.delta(),
                i.modifiers.shift,
                i.smooth_scroll_delta,
                i.zoom_delta(),
            )
        });
        if let Some(action) = gesture(
            response.dragged_by(egui::PointerButton::Secondary),
            response.dragged_by(egui::PointerButton::Middle),
            shift,
        ) {
            view = transform(view, action, delta, image.height());
        }
        if response.hovered() {
            if shift && pinch == 1.0 {
                view = transform(view, Gesture::Pan, scroll, image.height());
            } else {
                view = zoom(view, scroll.y, pinch);
            }
        }
        if view != before && valid_camera(view) {
            output.camera = Some(view);
        }
    }
    if can_pick && output.camera.is_none() && response.clicked_by(egui::PointerButton::Primary) {
        output.pick = response
            .interact_pointer_pos()
            .and_then(|position| coordinates(image, position));
    }
    let (held, focused, escape, position) = ui.input(|i| {
        (
            i.pointer.primary_down(),
            i.focused,
            i.key_pressed(egui::Key::Escape),
            i.pointer.interact_pos(),
        )
    });
    let started = response.drag_started_by(egui::PointerButton::Primary);
    let stopped = response.drag_stopped_by(egui::PointerButton::Primary);
    let cut = ui.input(|i| PointerCut {
        pressed: i.pointer.button_pressed(egui::PointerButton::Primary),
        released: i.pointer.button_released(egui::PointerButton::Primary),
        held,
        started,
        stopped,
    });
    if cut.pressed || cut.released || cut.started || cut.stopped {
        output.pointer_cut = Some(cut);
    }
    output.cancel = canceled(held, focused, escape, can_pick);
    if held && focused && !escape {
        output.held = position.map(|p| (p, image.height()));
        if can_pick && response.drag_started_by(egui::PointerButton::Primary) {
            output.begin = ui
                .input(|i| i.pointer.press_origin())
                .and_then(|p| coordinates(image, p).map(|xy| (p, xy, image.height())));
        }
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    fn camera() -> ViewCamera {
        ViewCamera {
            look_at: [0.0; 3],
            distance: 2.0,
            azimuth: 90.0,
            elevation: -20.0,
        }
    }
    #[test]
    fn letterboxing_and_pixel_scale_do_not_change_native_coordinates() {
        let rect = egui::Rect::from_min_size(egui::pos2(100.0, 200.0), egui::vec2(400.0, 200.0));
        assert_eq!(coordinates(rect, rect.center()), Some([0.5, 0.5]));
        assert_eq!(coordinates(rect, rect.left_top()), Some([0.0, 1.0]));
        assert_eq!(coordinates(rect, rect.right_bottom()), Some([1.0, 0.0]));
        assert_eq!(coordinates(rect, egui::pos2(99.0, 250.0)), None);
        assert_eq!(
            coordinates(
                egui::Rect::from_min_size(rect.min * 2.0, rect.size() * 2.0),
                rect.center() * 2.0
            ),
            Some([0.5, 0.5])
        );
    }
    #[test]
    fn gestures_reserve_primary_drag_and_camera_changes_are_bounded() {
        assert_eq!(gesture(false, false, false), None);
        assert_eq!(gesture(true, false, false), Some(Gesture::Orbit));
        assert_eq!(gesture(true, false, true), Some(Gesture::Pan));
        assert_eq!(gesture(false, true, false), Some(Gesture::Pan));
        assert_eq!(
            transform(camera(), Gesture::Orbit, egui::Vec2::ZERO, 100.0),
            camera()
        );
        let view = transform(camera(), Gesture::Orbit, egui::vec2(1e8, 1e8), 100.0);
        assert!(valid_camera(view));
        assert_eq!(zoom(camera(), 1e8, 1.0).distance, 0.01);
        assert_eq!(zoom(camera(), -1e8, 1.0).distance, 100_000.0);
        assert_eq!(zoom(camera(), 0.0, 2.0).distance, 1.0);
        assert_eq!(zoom(camera(), f32::NAN, 1.0), camera());
        assert!(!valid_camera(ViewCamera {
            distance: f64::NAN,
            ..camera()
        }));
        assert_ne!(
            transform(camera(), Gesture::Pan, egui::vec2(10.0, 5.0), 100.0).look_at,
            camera().look_at
        );
    }
}

#[cfg(test)]
mod drag_input_tests {
    use super::*;
    #[test]
    fn pinned_drag_projection_is_stable_and_bounded() {
        let start = egui::pos2(50.0, 50.0);
        let point = egui::pos2(70.0, 30.0);
        assert_eq!(drag_displacement(start, point, 100.0), [0.2, 0.2]);
        assert_eq!(drag_displacement(start, start, 100.0), [0.0; 2]);
        assert_eq!(
            drag_displacement(start, egui::pos2(1e8, -1e8), 100.0),
            [2.0; 2]
        );
        assert!(drag_displacement(start, point, 0.0)[0].is_nan());
        assert!(drag_displacement(start, egui::pos2(f32::NAN, 0.0), 100.0)[0].is_nan());
    }
    #[test]
    fn release_focus_escape_and_disabled_cancel_but_viewport_leave_does_not() {
        assert!(!canceled(true, true, false, true));
        assert!(canceled(false, true, false, true));
        assert!(canceled(true, false, false, true));
        assert!(canceled(true, true, true, true));
        assert!(canceled(true, true, false, false));
        // A held pointer may leave image bounds; update coordinates remain
        // relative to the pinned start rather than becoming a new native pick.
        assert!(
            coordinates(
                egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(100.0, 100.0)),
                egui::pos2(101.0, 50.0)
            )
            .is_none()
        );
        assert!(!canceled(true, true, false, true));
    }
}

#[cfg(test)]
mod event_cut_tests {
    use super::*;
    #[test]
    fn held_frames_begin_and_renew_but_atomic_release_cannot_leave_authority() {
        let ctx = egui::Context::default();
        let texture = ctx.load_texture(
            "event-cut",
            egui::ColorImage::filled([100, 100], egui::Color32::WHITE),
            Default::default(),
        );
        let mut time = 0.0;
        let mut frame = |events| {
            time += 0.1;
            let mut input = Input::default();
            let mut output = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(300.0, 300.0),
                    )),
                    time: Some(time),
                    events,
                    focused: true,
                    ..Default::default()
                },
                |ui| {
                    input = show(ui, &texture, None, true);
                },
            );
            output.textures_delta.clear();
            input
        };
        let start = egui::pos2(100.0, 100.0);
        let end = egui::pos2(180.0, 160.0);
        let button = |pos, pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        frame(Vec::new());
        frame(vec![egui::Event::PointerMoved(start), button(start, true)]);
        let held = frame(vec![egui::Event::PointerMoved(end)]);
        assert!(held.begin.is_some());
        assert!(held.held.is_some());
        assert!(!held.cancel);
        let stationary = frame(Vec::new());
        assert!(stationary.held.is_some());
        assert!(!stationary.cancel);
        assert!(frame(vec![button(end, false)]).cancel);
        let atomic = frame(vec![
            egui::Event::PointerMoved(start),
            button(start, true),
            egui::Event::PointerMoved(end),
            button(end, false),
        ]);
        assert!(atomic.cancel);
        assert!(atomic.begin.is_none());
        assert!(atomic.held.is_none());
        let cut = atomic.pointer_cut.unwrap();
        assert!(cut.pressed && cut.released && !cut.held);
    }
}
