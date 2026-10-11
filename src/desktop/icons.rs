//! Genuine Lucide glyph assets, vendored from lucide-static v1.54.0 (ISC,
//! see `resources/icons/LICENSE`).
//! The SVGs keep upstream 24-unit viewBox paths, caps and joins with the
//! approved demo's 1.7 SVG-unit stroke; `currentColor` is substituted per
//! state before loading, and each color variant is cached by URI.
use eframe::egui;

/// Logical glyph size inside a normal 30pt target.
pub(super) const GLYPH: f32 = 16.;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Icon {
    MousePointer2,
    Hand,
    Scan,
    Crosshair,
    House,
    CircleHelp,
    Play,
    Pause,
    StepForward,
    RotateCcw,
    Square,
    PanelLeft,
    PanelRight,
    ChevronDown,
    Plus,
    FolderOpen,
    Box,
    ArrowDownToLine,
    X,
    Terminal,
    Globe2,
}

impl Icon {
    /// Every mapped glyph, for regression coverage.
    #[cfg(test)]
    pub(super) const ALL: [Self; 21] = [
        Self::MousePointer2,
        Self::Hand,
        Self::Scan,
        Self::Crosshair,
        Self::House,
        Self::CircleHelp,
        Self::Play,
        Self::Pause,
        Self::StepForward,
        Self::RotateCcw,
        Self::Square,
        Self::PanelLeft,
        Self::PanelRight,
        Self::ChevronDown,
        Self::Plus,
        Self::FolderOpen,
        Self::Box,
        Self::ArrowDownToLine,
        Self::X,
        Self::Terminal,
        Self::Globe2,
    ];
    pub(super) fn slug(self) -> &'static str {
        match self {
            Self::MousePointer2 => "mouse-pointer-2",
            Self::Hand => "hand",
            Self::Scan => "scan",
            Self::Crosshair => "crosshair",
            Self::House => "house",
            Self::CircleHelp => "circle-help",
            Self::Play => "play",
            Self::Pause => "pause",
            Self::StepForward => "step-forward",
            Self::RotateCcw => "rotate-ccw",
            Self::Square => "square",
            Self::PanelLeft => "panel-left",
            Self::PanelRight => "panel-right",
            Self::ChevronDown => "chevron-down",
            Self::Plus => "plus",
            Self::FolderOpen => "folder-open",
            Self::Box => "box",
            Self::ArrowDownToLine => "arrow-down-to-line",
            Self::X => "x",
            Self::Terminal => "terminal",
            Self::Globe2 => "globe-2",
        }
    }
    pub(super) fn svg(self) -> &'static str {
        match self {
            Self::MousePointer2 => include_str!("../../resources/icons/mouse-pointer-2.svg"),
            Self::Hand => include_str!("../../resources/icons/hand.svg"),
            Self::Scan => include_str!("../../resources/icons/scan.svg"),
            Self::Crosshair => include_str!("../../resources/icons/crosshair.svg"),
            Self::House => include_str!("../../resources/icons/house.svg"),
            Self::CircleHelp => include_str!("../../resources/icons/circle-help.svg"),
            Self::Play => include_str!("../../resources/icons/play.svg"),
            Self::Pause => include_str!("../../resources/icons/pause.svg"),
            Self::StepForward => include_str!("../../resources/icons/step-forward.svg"),
            Self::RotateCcw => include_str!("../../resources/icons/rotate-ccw.svg"),
            Self::Square => include_str!("../../resources/icons/square.svg"),
            Self::PanelLeft => include_str!("../../resources/icons/panel-left.svg"),
            Self::PanelRight => include_str!("../../resources/icons/panel-right.svg"),
            Self::ChevronDown => include_str!("../../resources/icons/chevron-down.svg"),
            Self::Plus => include_str!("../../resources/icons/plus.svg"),
            Self::FolderOpen => include_str!("../../resources/icons/folder-open.svg"),
            Self::Box => include_str!("../../resources/icons/box.svg"),
            Self::ArrowDownToLine => {
                include_str!("../../resources/icons/arrow-down-to-line.svg")
            }
            Self::X => include_str!("../../resources/icons/x.svg"),
            Self::Terminal => include_str!("../../resources/icons/terminal.svg"),
            Self::Globe2 => include_str!("../../resources/icons/globe-2.svg"),
        }
    }
    fn colored_bytes(self, color: egui::Color32) -> (String, egui::load::Bytes) {
        let hex = format!("{:02X}{:02X}{:02X}", color.r(), color.g(), color.b());
        let uri = format!("lucide://{slug}-{hex}.svg", slug = self.slug());
        let colored = self
            .svg()
            .replace("currentColor", &format!("#{hex}"))
            .into_bytes();
        (uri, egui::load::Bytes::from(colored))
    }
    /// A state-colored image for this glyph, sized for a normal target cell.
    pub(super) fn image(self, color: egui::Color32) -> egui::Image<'static> {
        let (uri, bytes) = self.colored_bytes(color);
        egui::Image::from_bytes(uri, bytes)
            .fit_to_exact_size(egui::vec2(GLYPH, GLYPH))
            .sense(egui::Sense::hover())
    }
    /// Paint the glyph centered in `rect`, at `size` logical points.
    /// Painting through the painter keeps the glyph outside layout, so host
    /// toolbars and cells measure exactly as before.
    pub(super) fn paint_at(
        self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        color: egui::Color32,
        size: f32,
    ) {
        self.paint_at_rotated(ui, rect, color, size, false);
    }
    /// Paint the genuine glyph rotated a quarter turn counter-clockwise.
    /// Rotation is presentation of the authentic asset, not a new glyph.
    pub(super) fn paint_at_rotated(
        self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        color: egui::Color32,
        size: f32,
        rotate: bool,
    ) {
        let (uri, bytes) = self.colored_bytes(color);
        // Register the colored SVG bytes under the URI, then resolve the
        // rasterized texture through the installed image loaders.
        ui.ctx().include_bytes(uri.clone(), bytes);
        let texture = ui.ctx().try_load_texture(
            &uri,
            egui::TextureOptions::LINEAR,
            egui::load::SizeHint::Width(48),
        );
        if let Ok(egui::load::TexturePoll::Ready { texture }) = texture {
            let target = egui::Rect::from_center_size(rect.center(), egui::vec2(size, size));
            if rotate {
                // A quarter-turn of the rasterized quad: the glyph's bottom
                // edge is mapped onto the target's right edge, so a
                // down-chevron renders pointing right.
                let v = |pos: egui::Pos2, uv: egui::Pos2| egui::epaint::Vertex {
                    pos,
                    uv,
                    color: egui::Color32::WHITE,
                };
                let mesh = egui::Mesh {
                    texture_id: texture.id,
                    indices: vec![0, 1, 2, 2, 1, 3],
                    vertices: vec![
                        v(target.left_top(), egui::pos2(1., 0.)),
                        v(target.right_top(), egui::pos2(1., 1.)),
                        v(target.left_bottom(), egui::pos2(0., 0.)),
                        v(target.right_bottom(), egui::pos2(0., 1.)),
                    ],
                };
                ui.painter()
                    .add(egui::Shape::Mesh(std::sync::Arc::new(mesh)));
            } else {
                ui.painter().image(
                    texture.id,
                    target,
                    egui::Rect::from_min_max(egui::pos2(0., 0.), egui::pos2(1., 1.)),
                    egui::Color32::WHITE,
                );
            }
        }
    }
}
