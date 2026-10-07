//! Latched cancellation is independent of the bounded interaction queue.
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[derive(Default)]
struct State {
    cancelled: AtomicBool,
    changed: tokio::sync::Notify,
}

#[derive(Clone, Default)]
pub(crate) struct Cancellation(Arc<State>);

impl Cancellation {
    pub(crate) fn cancel(&self) {
        self.0.cancelled.store(true, Ordering::Release);
        self.0.changed.notify_waiters();
    }

    pub(crate) fn is_cancelled(&self) -> bool {
        self.0.cancelled.load(Ordering::Acquire)
    }

    pub(crate) fn check(&self) -> Result<(), String> {
        if self.is_cancelled() {
            Err("Operation cancelled".into())
        } else {
            Ok(())
        }
    }

    pub(crate) async fn wait(&self) {
        loop {
            let changed = self.0.changed.notified();
            tokio::pin!(changed);
            changed.as_mut().enable();
            if self.is_cancelled() {
                return;
            }
            changed.await;
        }
    }
}
