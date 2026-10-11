//! Modal focus stays inside the active surface and returns to its invoking control.
use eframe::egui;
#[derive(Default)]
pub(super) struct Focus {
    active: Option<egui::Id>,
    restore: Option<egui::Id>,
    pending_restore: Option<egui::Id>,
    prior: Option<egui::Id>,
    entering: bool,
    direction: Option<bool>,
}
impl Focus {
    pub fn begin(&mut self, ctx: &egui::Context, id: egui::Id) {
        self.entering = self.active != Some(id);
        self.prior = ctx.memory(|m| m.focused());
        if self.entering {
            self.pending_restore = None;
            self.restore = self.prior;
            self.active = Some(id);
        }
        self.direction = if egui::Popup::is_any_open(ctx) {
            None
        } else {
            ctx.input_mut(|i| {
                if i.consume_key(egui::Modifiers::SHIFT, egui::Key::Tab) {
                    Some(false)
                } else if i.consume_key(egui::Modifiers::NONE, egui::Key::Tab) {
                    Some(true)
                } else {
                    None
                }
            })
        };
    }
    pub fn finish(&self, ctx: &egui::Context, ids: &[egui::Id]) {
        #[cfg(test)]
        ctx.data_mut(|d| d.insert_temp(egui::Id::new("modal-focus-probes"), ids.to_vec()));
        if ids.is_empty() || egui::Popup::is_any_open(ctx) {
            return;
        }
        let desired = if let Some(forward) = self.direction {
            let index = self.prior.and_then(|id| ids.iter().position(|x| *x == id));
            Some(
                ids[match (index, forward) {
                    (Some(i), true) => (i + 1) % ids.len(),
                    (Some(i), false) => (i + ids.len() - 1) % ids.len(),
                    (None, true) => 0,
                    (None, false) => ids.len() - 1,
                }],
            )
        } else if self.entering {
            ids.last().copied()
        } else if ctx
            .memory(|m| m.focused())
            .is_none_or(|id| !ids.contains(&id))
        {
            ids.first().copied()
        } else {
            None
        };
        if let Some(id) = desired {
            ctx.memory_mut(|m| m.request_focus(id));
        }
    }
    pub fn restore(&mut self, ctx: &egui::Context) {
        if self.active.take().is_some() {
            // The background is still disabled on the closing modal frame.
            // Retain the request until its enabled invoker can accept focus.
            self.pending_restore = self.restore.take();
            if let Some(id) = self.pending_restore {
                ctx.memory_mut(|m| m.request_focus(id));
            }
        } else if let Some(id) = self.pending_restore {
            if ctx.memory(|m| m.focused()).is_some() {
                self.pending_restore = None;
            } else {
                ctx.memory_mut(|m| m.request_focus(id));
            }
        }
    }
}
