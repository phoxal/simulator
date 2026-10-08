//! Desktop presentation only; native scene colors and execution are independent.
use eframe::egui::{self, Color32, Stroke};

pub(super) const BACKGROUND: Color32 = Color32::from_rgb(17, 24, 27);
pub(super) const SURFACE: Color32 = Color32::from_rgb(26, 37, 42);
const RAISED: Color32 = Color32::from_rgb(35, 52, 58);
const TEXT: Color32 = Color32::from_rgb(237, 244, 242);
pub(super) const MUTED: Color32 = Color32::from_rgb(177, 194, 197);
const BORDER: Color32 = Color32::from_rgb(113, 136, 141);
const ACCENT: Color32 = Color32::from_rgb(114, 222, 211);
const SELECTED: Color32 = Color32::from_rgb(36, 70, 65);
pub(super) const SUCCESS: Color32 = Color32::from_rgb(139, 221, 173);
pub(super) const WARNING: Color32 = Color32::from_rgb(244, 211, 94);
pub(super) const ERROR: Color32 = Color32::from_rgb(255, 156, 156);

pub(super) fn apply(context: &egui::Context) {
    let mut style = (*context.style_of(egui::Theme::Dark)).clone();
    let mut visuals = egui::Visuals::dark();
    visuals.override_text_color = None;
    visuals.weak_text_color = Some(MUTED);
    visuals.panel_fill = BACKGROUND;
    visuals.window_fill = SURFACE;
    visuals.window_stroke = Stroke::new(1.0, BORDER);
    visuals.extreme_bg_color = BACKGROUND;
    visuals.text_edit_bg_color = Some(BACKGROUND);
    visuals.code_bg_color = SURFACE;
    visuals.faint_bg_color = SURFACE;
    visuals.hyperlink_color = ACCENT;
    visuals.warn_fg_color = WARNING;
    visuals.error_fg_color = ERROR;
    visuals.selection.bg_fill = SELECTED;
    visuals.selection.stroke = Stroke::new(1.0, ACCENT);
    visuals.widgets.noninteractive.bg_fill = SURFACE;
    visuals.widgets.noninteractive.weak_bg_fill = SURFACE;
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(57, 76, 82));
    visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
    for widget in [
        &mut visuals.widgets.inactive,
        &mut visuals.widgets.hovered,
        &mut visuals.widgets.active,
        &mut visuals.widgets.open,
    ] {
        widget.bg_fill = RAISED;
        widget.weak_bg_fill = RAISED;
        widget.fg_stroke = Stroke::new(1.0, TEXT);
        widget.bg_stroke = Stroke::new(1.0, BORDER);
        widget.corner_radius = egui::CornerRadius::same(4);
        widget.expansion = 0.0;
    }
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT);
    // Egui uses active visuals for keyboard focus as well as an active press.
    // White outline stays distinct from the teal selected fill/text.
    visuals.widgets.active.bg_stroke = Stroke::new(2.0, Color32::WHITE);
    visuals.widgets.open.bg_stroke = Stroke::new(1.0, ACCENT);
    // Preserve disabled-state opacity without making prerequisites unreadable.
    visuals.disabled_alpha = 0.75;
    style.visuals = visuals;
    style.spacing.item_spacing = egui::vec2(8.0, 8.0);
    style.spacing.button_padding = egui::vec2(10.0, 6.0);
    style
        .text_styles
        .insert(egui::TextStyle::Heading, egui::FontId::proportional(18.0));
    context.set_style_of(egui::Theme::Dark, style);
    context.set_theme(egui::Theme::Dark);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn luminance(color: Color32) -> f64 {
        let channel = |value: u8| {
            let value = f64::from(value) / 255.0;
            if value <= 0.04045 {
                value / 12.92
            } else {
                ((value + 0.055) / 1.055).powf(2.4)
            }
        };
        0.2126 * channel(color.r()) + 0.7152 * channel(color.g()) + 0.0722 * channel(color.b())
    }
    fn contrast(text: Color32, background: Color32) -> f64 {
        let (a, b) = (luminance(text), luminance(background));
        (a.max(b) + 0.05) / (a.min(b) + 0.05)
    }

    #[test]
    fn presentation_roles_remain_readable_and_focus_is_not_selection_or_status() {
        let context = egui::Context::default();
        apply(&context);
        let style = context.style_of(egui::Theme::Dark);
        for background in [
            style.visuals.panel_fill,
            style.visuals.window_fill,
            style.visuals.widgets.inactive.bg_fill,
        ] {
            for text in [
                style.visuals.text_color(),
                style.visuals.weak_text_color(),
                SUCCESS,
                WARNING,
                ERROR,
            ] {
                assert!(
                    contrast(text, background) >= 4.5,
                    "{text:?} on {background:?}"
                );
            }
        }
        assert!(
            contrast(
                style.visuals.selection.stroke.color,
                style.visuals.selection.bg_fill
            ) >= 4.5
        );
        let disabled = style.visuals.disable(style.visuals.text_color());
        let alpha = f64::from(disabled.a()) / 255.0;
        let blend = |fg: u8, bg: u8| {
            (f64::from(fg) + f64::from(bg) * (1.0 - alpha))
                .round()
                .clamp(0.0, 255.0) as u8
        };
        let background = style.visuals.widgets.inactive.bg_fill;
        let painted = Color32::from_rgb(
            blend(disabled.r(), background.r()),
            blend(disabled.g(), background.g()),
            blend(disabled.b(), background.b()),
        );
        assert!(
            contrast(painted, background) >= 4.5,
            "disabled labels must remain readable"
        );
        let focus = style.visuals.widgets.active.bg_stroke;
        assert!(focus.width > style.visuals.widgets.inactive.bg_stroke.width);
        assert_ne!(focus.color, style.visuals.selection.stroke.color);
        assert_ne!(style.visuals.selection.stroke.color, SUCCESS);
    }
}
