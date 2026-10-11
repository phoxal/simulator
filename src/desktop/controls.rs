//! Shared native control geometry and genuine Lucide icon cells.
use super::{icons, theme};
use eframe::egui;

pub(super) use icons::Icon;
#[must_use]
pub(super) fn button(
    ui: &mut egui::Ui,
    icon: Icon,
    name: &str,
    selected: bool,
    enabled: bool,
    prerequisite: &str,
    primary: bool,
) -> egui::Response {
    let size = control_size(ui);
    let mut spec = egui::Button::new("").min_size(egui::vec2(size, size));
    if primary {
        // The primary action carries the teal emphasis even while ready; this
        // is action emphasis, never evidence that execution is running.
        spec = if enabled {
            spec.fill(theme::SELECTED_FILL)
        } else {
            spec.fill(theme::RAISED_FILL)
        };
    }
    let response = ui.add_enabled(enabled, spec.selected(selected));
    #[cfg(test)]
    ui.data_mut(|d| {
        let id = egui::Id::new("control-probes");
        let mut values = d
            .get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(id)
            .unwrap_or_default();
        values.push((
            name.to_owned(),
            response.rect,
            response.id,
            selected,
            primary,
            enabled,
        ));
        d.insert_temp(id, values);
    });
    response.widget_info(|| {
        egui::WidgetInfo::selected(egui::WidgetType::Button, enabled, selected, name)
    });
    let color = if enabled {
        if primary || selected {
            theme::ACCENT_TEXT
        } else if response.hovered() {
            theme::TEXT
        } else {
            theme::MUTED
        }
    } else {
        theme::DISABLED
    };
    let p = ui.painter();
    if primary && enabled && response.hovered() && !response.is_pointer_button_down_on() {
        p.rect_filled(
            response.rect,
            egui::CornerRadius::same(5),
            theme::SELECTED_HOVER,
        );
    }
    icons::Icon::paint_at(icon, ui, response.rect, color, icons::GLYPH);
    let help = if enabled || prerequisite.is_empty() {
        name.to_owned()
    } else {
        format!("{name}: {prerequisite}")
    };
    if response.has_focus() {
        response.clone().show_tooltip_text(&help);
    }
    response.on_hover_text(&help).on_disabled_hover_text(help)
}
pub(super) fn control_size(ui: &egui::Ui) -> f32 {
    theme::CONTROL
        .max(
            ui.text_style_height(&egui::TextStyle::Body)
                + ui.text_style_height(&egui::TextStyle::Small),
        )
        .max(ui.spacing().interact_size.y)
}
pub(super) fn toolbar(
    ui: &mut egui::Ui,
    width: f32,
    content: impl FnOnce(&mut egui::Ui),
) -> egui::InnerResponse<()> {
    egui::Frame::new()
        .fill(theme::SURFACE)
        .stroke(egui::Stroke::new(theme::BORDER_WIDTH, theme::LINE))
        .inner_margin(theme::PADDING as i8)
        .corner_radius(theme::RADIUS)
        .show(ui, |ui| {
            ui.set_width(width - 2. * (theme::PADDING + theme::BORDER_WIDTH));
            ui.spacing_mut().item_spacing = egui::vec2(theme::GAP, theme::GAP);
            ui.spacing_mut().button_padding = egui::vec2(0., 0.);
            ui.horizontal_wrapped(content);
        })
}
