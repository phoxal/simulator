//! Desktop presentation only; native scene colors and execution are independent.
use eframe::egui::{self, Color32, Stroke};

pub(super) const CONTROL: f32 = 30.;
pub(super) const GAP: f32 = 4.;
pub(super) const PADDING: f32 = 5.;
pub(super) const BORDER_WIDTH: f32 = 1.;
pub(super) const RADIUS: u8 = 7;
pub(super) const INSET: f32 = 16.;
pub(super) const STACK_GAP: f32 = 8.;
pub(super) const NOTICE_GAP: f32 = 12.;
pub(super) const BACKGROUND: Color32 = Color32::from_rgb(17, 24, 27);
pub(super) const SURFACE: Color32 = Color32::from_rgb(26, 37, 42);
const RAISED: Color32 = Color32::from_rgb(35, 52, 58);
pub(super) const RAISED_FILL: Color32 = RAISED;
pub(super) const TEXT: Color32 = Color32::from_rgb(237, 244, 242);
pub(super) const MUTED: Color32 = Color32::from_rgb(177, 194, 197);
/// Quiet separation for groups, panels and strips.
pub(super) const LINE: Color32 = Color32::from_rgb(66, 88, 94);
/// Stronger boundary for explicit interactive outlines (Import, pickers).
pub(super) const BORDER: Color32 = Color32::from_rgb(113, 136, 141);
const ACCENT: Color32 = Color32::from_rgb(114, 222, 211);
const SELECTED: Color32 = Color32::from_rgb(36, 70, 65);
pub(super) const SELECTED_FILL: Color32 = SELECTED;
pub(super) const SELECTED_HOVER: Color32 = Color32::from_rgb(48, 91, 84);
pub(super) const ACCENT_TEXT: Color32 = ACCENT;
pub(super) const DISABLED: Color32 = Color32::from_rgb(152, 169, 172);
pub(super) const SUCCESS: Color32 = Color32::from_rgb(139, 221, 173);
pub(super) const WARNING: Color32 = Color32::from_rgb(244, 211, 94);
pub(super) const ERROR: Color32 = Color32::from_rgb(255, 156, 156);

pub(super) fn apply(context: &egui::Context) {
    egui_extras::install_image_loaders(context);
    let mut style = (*context.style_of(egui::Theme::Dark)).clone();
    let mut visuals = egui::Visuals::dark();
    visuals.override_text_color = None;
    visuals.weak_text_color = Some(MUTED);
    visuals.panel_fill = BACKGROUND;
    visuals.window_fill = SURFACE;
    visuals.window_stroke = Stroke::new(1.0, LINE);
    visuals.extreme_bg_color = BACKGROUND;
    visuals.text_edit_bg_color = Some(BACKGROUND);
    visuals.code_bg_color = SURFACE;
    visuals.faint_bg_color = SURFACE;
    visuals.hyperlink_color = ACCENT;
    visuals.warn_fg_color = WARNING;
    visuals.error_fg_color = ERROR;
    visuals.selection.bg_fill = SELECTED;
    visuals.selection.stroke = Stroke::new(1.0, ACCENT);
    // Quiet resting controls: transparent fill without individual borders.
    // The containing toolbar, panel or strip supplies the surface and line.
    visuals.widgets.noninteractive.bg_fill = SURFACE;
    visuals.widgets.noninteractive.weak_bg_fill = SURFACE;
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, LINE);
    visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
    for widget in [
        &mut visuals.widgets.inactive,
        &mut visuals.widgets.hovered,
        &mut visuals.widgets.active,
        &mut visuals.widgets.open,
    ] {
        widget.bg_fill = Color32::TRANSPARENT;
        widget.weak_bg_fill = Color32::TRANSPARENT;
        widget.fg_stroke = Stroke::new(1.0, TEXT);
        widget.bg_stroke = Stroke::NONE;
        widget.corner_radius = egui::CornerRadius::same(5);
        widget.expansion = 0.0;
    }
    // Hover raises a local surface without changing layout or borders.
    visuals.widgets.hovered.bg_fill = RAISED;
    visuals.widgets.hovered.weak_bg_fill = RAISED;
    // Egui uses active visuals for keyboard focus as well as an active press.
    // White outline stays distinct from the teal selected fill/text.
    visuals.widgets.active.bg_stroke = Stroke::new(2.0, Color32::WHITE);
    visuals.widgets.open.bg_fill = RAISED;
    // Preserve disabled-state opacity without making prerequisites unreadable.
    visuals.disabled_alpha = 0.75;
    style.visuals = visuals;
    style.spacing.item_spacing = egui::vec2(8.0, 8.0);
    style.spacing.button_padding = egui::vec2(7.0, 5.0);
    // Compact typographic roles: 13pt base, 12pt labels, 11pt metadata.
    style
        .text_styles
        .insert(egui::TextStyle::Body, egui::FontId::proportional(13.0));
    style
        .text_styles
        .insert(egui::TextStyle::Button, egui::FontId::proportional(12.0));
    style
        .text_styles
        .insert(egui::TextStyle::Small, egui::FontId::proportional(11.0));
    style
        .text_styles
        .insert(egui::TextStyle::Heading, egui::FontId::proportional(18.0));
    style
        .text_styles
        .insert(egui::TextStyle::Monospace, egui::FontId::monospace(12.0));
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

    #[test]
    fn resting_controls_are_quiet_while_hover_selected_and_focus_stay_distinct() {
        let context = egui::Context::default();
        apply(&context);
        let widgets = &context.style_of(egui::Theme::Dark).visuals.widgets;
        // Resting icon cells are transparent with no individual border.
        assert_eq!(widgets.inactive.bg_fill, Color32::TRANSPARENT);
        assert_eq!(widgets.inactive.bg_stroke, Stroke::NONE);
        // Hover raises a local surface without borders.
        assert_ne!(widgets.hovered.bg_fill, Color32::TRANSPARENT);
        assert_eq!(widgets.hovered.bg_stroke, Stroke::NONE);
        // Keyboard focus keeps a white outline distinct from teal selection.
        assert_eq!(widgets.active.bg_stroke.color, Color32::WHITE);
        let selection = &context.style_of(egui::Theme::Dark).visuals.selection;
        assert_eq!(selection.bg_fill, SELECTED_FILL);
        assert_eq!(selection.stroke.color, ACCENT_TEXT);
        // Teal selection stays distinct from operational status colors.
        assert_ne!(selection.stroke.color, SUCCESS);
        assert_ne!(selection.stroke.color, WARNING);
        assert_ne!(selection.stroke.color, ERROR);
    }
}
