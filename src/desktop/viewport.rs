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
    pub pan: bool,
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
use crate::app::scene::valid_camera;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Gesture {
    Orbit,
    Pan,
}
fn gesture(secondary: bool, middle: bool, _shift: bool) -> Option<Gesture> {
    if middle || secondary {
        Some(Gesture::Pan)
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
    manipulate: bool,
    captured: &[egui::Rect],
    revision: u64,
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
    let blocked = ui.input(|i| i.pointer.interact_pos()).is_some_and(|p| {
        captured.iter().any(|r| r.contains(p))
            || ui
                .ctx()
                .layer_id_at(p)
                .is_some_and(|layer| layer != ui.layer_id())
    });
    let can_pick = can_pick && !blocked;
    let camera = if blocked { None } else { camera };
    let gesture_id = ui.id().with("pointer-gestures");
    let (focused, escape, shift) = ui.input(|i| {
        (
            i.focused,
            i.key_pressed(egui::Key::Escape),
            i.modifiers.shift,
        )
    });
    let mut gestures = ui
        .data_mut(|d| d.get_temp::<[Option<(bool, bool, bool, u64)>; 3]>(gesture_id))
        .unwrap_or_default();
    for (index, button) in [
        egui::PointerButton::Primary,
        egui::PointerButton::Secondary,
        egui::PointerButton::Middle,
    ]
    .into_iter()
    .enumerate()
    {
        let (pressed, held, released) = ui.input(|i| {
            (
                i.pointer.button_pressed(button),
                i.pointer.button_down(button),
                i.pointer.button_released(button),
            )
        });
        if !held && !released {
            gestures[index] = None;
        }
        // Navigation needs a valid camera, while native picks and manipulation
        // additionally require the exact presented frame. Presentation lag must
        // not cancel a camera-only gesture.
        if pressed
            && response.hovered()
            && !blocked
            && focused
            && !escape
            && (camera.is_some() || can_pick)
        {
            gestures[index] = Some((manipulate, shift, false, revision));
        }
        if let Some((mode, _, cancelled, identity)) = gestures[index].as_mut()
            && (*mode != manipulate
                || *identity != revision
                || !focused
                || escape
                || blocked
                || (camera.is_none() && !can_pick)
                || (index == 0 && *mode && !can_pick))
        {
            *cancelled = true;
        }
    }
    let primary = gestures[0];
    let manipulating = primary.is_some_and(|(mode, _, cancelled, _)| mode && !cancelled);
    let mut retained = gestures;
    for (index, button) in [
        egui::PointerButton::Primary,
        egui::PointerButton::Secondary,
        egui::PointerButton::Middle,
    ]
    .into_iter()
    .enumerate()
    {
        if !ui.input(|i| i.pointer.button_down(button)) {
            retained[index] = None;
        }
    }
    ui.data_mut(|d| d.insert_temp(gesture_id, retained));
    let mut output = Input::default();
    #[cfg(test)]
    {
        output.canvas = Some(canvas);
    }
    if let Some(mut view) = camera.filter(|_| focused && !escape) {
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
            response.dragged_by(egui::PointerButton::Secondary)
                && gestures[1].is_some_and(|(_, _, c, _)| !c),
            response.dragged_by(egui::PointerButton::Middle)
                && gestures[2].is_some_and(|(_, _, c, _)| !c),
            shift,
        ) {
            output.pan = action == Gesture::Pan;
            view = transform(view, action, delta, image.height());
        }
        if response.dragged_by(egui::PointerButton::Primary)
            && primary.is_some_and(|(mode, _, cancelled, _)| !mode && !cancelled)
        {
            let pan = primary.is_some_and(|(_, shift, _, _)| shift);
            output.pan = pan;
            view = transform(
                view,
                if pan { Gesture::Pan } else { Gesture::Orbit },
                delta,
                image.height(),
            );
        }
        if response.hovered() {
            if shift && pinch == 1.0 {
                output.pan = scroll != egui::Vec2::ZERO;
                view = transform(view, Gesture::Pan, scroll, image.height());
            } else {
                view = zoom(view, scroll.y, pinch);
            }
        }
        if view != before && valid_camera(view) {
            output.camera = Some(view);
        }
    }
    if can_pick
        && primary.is_some_and(|(_, _, cancelled, _)| !cancelled)
        && output.camera.is_none()
        && response.clicked_by(egui::PointerButton::Primary)
    {
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
    output.cancel = canceled(held, focused, escape, can_pick) || !manipulating;
    if manipulating && held && focused && !escape {
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
    pub(super) fn camera() -> ViewCamera {
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
        assert_eq!(gesture(true, false, false), Some(Gesture::Pan));
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
                    input = show(ui, &texture, None, true, true, &[], 0);
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

#[cfg(test)]
mod navigation_fence_tests {
    use super::*;
    struct Driver {
        ctx: egui::Context,
        texture: egui::TextureHandle,
        view: ViewCamera,
        time: f64,
    }
    impl Driver {
        fn new() -> Self {
            let ctx = egui::Context::default();
            let texture = ctx.load_texture(
                "navigation",
                egui::ColorImage::filled([100, 100], egui::Color32::WHITE),
                Default::default(),
            );
            Self {
                ctx,
                texture,
                view: super::tests::camera(),
                time: 0.,
            }
        }
        fn frame(
            &mut self,
            events: Vec<egui::Event>,
            focus: bool,
            blocked: bool,
            revision: u64,
            pick: bool,
            manipulate: bool,
        ) -> Input {
            self.time += 0.1;
            let mut input = Input::default();
            let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(300., 300.));
            let mut output = self.ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(screen),
                    time: Some(self.time),
                    events,
                    focused: focus,
                    ..Default::default()
                },
                |ui| {
                    input = show(
                        ui,
                        &self.texture,
                        Some(self.view),
                        pick,
                        manipulate,
                        if blocked {
                            std::slice::from_ref(&screen)
                        } else {
                            &[]
                        },
                        revision,
                    );
                },
            );
            output.textures_delta.clear();
            if let Some(view) = input.camera {
                self.view = view;
            }
            input
        }
    }
    fn button(button: egui::PointerButton, pos: egui::Pos2, pressed: bool) -> egui::Event {
        egui::Event::PointerButton {
            pos,
            button,
            pressed,
            modifiers: Default::default(),
        }
    }
    #[test]
    fn every_button_cancellation_requires_release_and_fresh_press() {
        for b in [
            egui::PointerButton::Primary,
            egui::PointerButton::Secondary,
            egui::PointerButton::Middle,
        ] {
            for cause in 0..5 {
                let mut d = Driver::new();
                d.frame(vec![], true, false, 0, true, false);
                let start = egui::pos2(100., 100.);
                let moved = egui::pos2(140., 120.);
                d.frame(
                    vec![egui::Event::PointerMoved(start), button(b, start, true)],
                    true,
                    false,
                    0,
                    true,
                    false,
                );
                assert!(
                    d.frame(
                        vec![egui::Event::PointerMoved(moved)],
                        true,
                        false,
                        0,
                        true,
                        false
                    )
                    .camera
                    .is_some(),
                    "ordinary {b:?} navigation"
                );
                let escape = egui::Event::Key {
                    key: egui::Key::Escape,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: Default::default(),
                };
                d.frame(
                    if cause == 2 { vec![escape] } else { vec![] },
                    cause != 1,
                    cause == 0,
                    if cause == 3 { 1 } else { 0 },
                    true,
                    cause == 4,
                );
                let before = d.view;
                assert!(
                    d.frame(
                        vec![egui::Event::PointerMoved(egui::pos2(180., 150.))],
                        true,
                        false,
                        if cause == 3 { 1 } else { 0 },
                        true,
                        false
                    )
                    .camera
                    .is_none(),
                    "{b:?} resumed canceled cause {cause}"
                );
                assert_eq!(d.view, before);
                let released = d.frame(
                    vec![button(b, moved, false)],
                    true,
                    false,
                    if cause == 3 { 1 } else { 0 },
                    true,
                    false,
                );
                assert!(
                    released.pick.is_none(),
                    "canceled release selected underneath"
                );
                d.frame(
                    vec![egui::Event::PointerMoved(start), button(b, start, true)],
                    true,
                    false,
                    if cause == 3 { 1 } else { 0 },
                    true,
                    false,
                );
                assert!(
                    d.frame(
                        vec![egui::Event::PointerMoved(moved)],
                        true,
                        false,
                        if cause == 3 { 1 } else { 0 },
                        true,
                        false
                    )
                    .camera
                    .is_some(),
                    "fresh {b:?} navigation did not recover"
                );
            }
        }
    }
    #[test]
    fn cancelled_primary_click_release_never_picks_underneath() {
        for cause in 0..3 {
            let mut d = Driver::new();
            d.frame(vec![], true, false, 0, true, false);
            let point = egui::pos2(100., 100.);
            d.frame(
                vec![
                    egui::Event::PointerMoved(point),
                    button(egui::PointerButton::Primary, point, true),
                ],
                true,
                false,
                0,
                true,
                false,
            );
            d.frame(
                if cause == 0 {
                    vec![egui::Event::Key {
                        key: egui::Key::Escape,
                        physical_key: None,
                        pressed: true,
                        repeat: false,
                        modifiers: Default::default(),
                    }]
                } else {
                    vec![]
                },
                cause != 1,
                cause == 2,
                0,
                true,
                false,
            );
            let released = d.frame(
                vec![button(egui::PointerButton::Primary, point, false)],
                true,
                false,
                0,
                true,
                false,
            );
            assert!(
                released.pick.is_none(),
                "canceled primary release cause {cause} picked underneath"
            );
            d.frame(
                vec![button(egui::PointerButton::Primary, point, true)],
                true,
                false,
                0,
                true,
                false,
            );
            assert!(
                d.frame(
                    vec![button(egui::PointerButton::Primary, point, false)],
                    true,
                    false,
                    0,
                    true,
                    false
                )
                .pick
                .is_some(),
                "fresh click lost ordinary picking"
            );
        }
    }
    #[test]
    fn stale_presented_camera_does_not_cancel_continuous_navigation() {
        let mut d = Driver::new();
        d.frame(vec![], true, false, 0, true, false);
        let start = egui::pos2(100., 100.);
        d.frame(
            vec![
                egui::Event::PointerMoved(start),
                button(egui::PointerButton::Primary, start, true),
            ],
            true,
            false,
            0,
            true,
            false,
        );
        assert!(
            d.frame(
                vec![egui::Event::PointerMoved(egui::pos2(140., 120.))],
                true,
                false,
                0,
                true,
                false
            )
            .camera
            .is_some()
        );
        assert!(
            d.frame(
                vec![egui::Event::PointerMoved(egui::pos2(180., 150.))],
                true,
                false,
                0,
                false,
                false
            )
            .camera
            .is_some(),
            "camera-only orbit canceled while native presentation caught up"
        );
    }
}
