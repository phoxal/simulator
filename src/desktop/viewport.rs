use eframe::egui;
use crate::mujoco::ViewCamera;

pub(super) fn show(
    ui: &mut egui::Ui,
    texture: &egui::TextureHandle,
    camera: Option<ViewCamera>,
) -> Option<ViewCamera> {
    let available = (ui.available_size() - egui::vec2(0.0, 32.0)).max(egui::vec2(1.0, 1.0));
    let dimensions = texture.size_vec2();
    let scale = (available.x / dimensions.x).min(available.y / dimensions.y);
    let size = dimensions * scale;
    let canvas = egui::Rect::from_min_size(ui.cursor().min, available);
    ui.painter()
        .rect_filled(canvas, 12.0, egui::Color32::from_rgb(11, 14, 19));
    let response = ui
        .allocate_ui_with_layout(
            available,
            egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
            |ui| {
                ui.add(
                    egui::Image::new(texture)
                        .fit_to_exact_size(size)
                        .sense(egui::Sense::drag()),
                )
            },
        )
        .inner;
    let mut view = camera?;
    let mut changed = false;
    if response.dragged() {
        let delta = ui.input(|i| i.pointer.delta());
        view.azimuth -= f64::from(delta.x) * 0.4;
        view.elevation = (view.elevation + f64::from(delta.y) * 0.4).clamp(-89.0, 89.0);
        changed = true;
    }
    if response.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll != 0.0 {
            view.distance =
                (view.distance * (-f64::from(scroll) * 0.003).exp()).clamp(0.01, 100_000.0);
            changed = true;
        }
    }
    changed.then_some(view)
}
