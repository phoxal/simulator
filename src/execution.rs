//! One boundary coordinator for finite headless runs and desktop controls.

use crate::mujoco::{Model, StateSnapshot};
use crate::{
    authority::{AuthorityState, SimulationTransport},
    desktop::{Command, Worker},
    native_provider::ComponentProvider,
    remote::RemoteSceneRun,
};
use std::time::{Duration, Instant};
use tokio::sync::mpsc::error::TryRecvError;

mod interaction;
mod timing;
mod viewport;
pub(crate) use timing::Pacing;
use timing::Timing;

type Run<T> = RemoteSceneRun<T, ComponentProvider>;

pub(super) async fn drive<T: SimulationTransport>(
    run: &mut Run<T>,
    model: &Model,
    requested_steps: u64,
    mut desktop: Option<Worker>,
    auto_run: bool,
    collect_every_boundary: bool,
) -> Result<Vec<StateSnapshot>, String> {
    let mut running = desktop.is_none() || auto_run;
    let cancel = desktop.as_ref().map(|worker| worker.cancel.clone());
    let mut snapshots = History::new(
        collect_every_boundary || desktop.is_none(),
        run.state().clone(),
    );
    let mut viewport = if desktop.is_some() {
        Some(viewport::Viewport::new(
            model,
            &run.provenance().execution_id,
            run.generation(),
        )?)
    } else {
        None
    };
    let mut interaction = interaction::Interaction::default();
    let mut frame_due = true;
    let mut last_frame = Instant::now();
    let mut last_renewal = Instant::now();
    let mut interrupt = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())
        .map_err(|e| e.to_string())?;
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        .map_err(|e| e.to_string())?;
    let epoch = Instant::now();
    let mut timing = Timing::new(if desktop.is_some() {
        Pacing::Realtime
    } else {
        Pacing::Fast
    });
    let mut pending = None;
    if running {
        timing.resume(epoch.elapsed(), sim_time(run));
    }
    'execution: loop {
        if desktop
            .as_ref()
            .is_some_and(|worker| worker.cancel.is_cancelled())
        {
            break;
        }
        let stop_signal = tokio::select! {
            biased;
            _ = interrupt.recv() => true,
            _ = terminate.recv() => true,
            _ = std::future::ready(()) => false,
        };
        if stop_signal {
            if let Some(worker) = &desktop {
                worker
                    .display
                    .lock()
                    .map_err(|_| "desktop state lock poisoned")?
                    .close_requested = true;
            }
            break;
        }
        let mut single_step = false;
        if let Some(worker) = &mut desktop {
            loop {
                let command = pending
                    .take()
                    .map(Ok)
                    .unwrap_or_else(|| worker.commands.try_recv());
                match command {
                    Ok(Command::Run) => {
                        if !running {
                            interaction.cancel(run, &worker.display)?;
                        }
                        running = true;
                        timing.resume(epoch.elapsed(), sim_time(run));
                    }
                    Ok(Command::Pause) => {
                        interaction.cancel(run, &worker.display)?;
                        running = false;
                        timing.pause(epoch.elapsed());
                    }
                    Ok(Command::SetPacing(mode)) => {
                        timing.set_mode(mode, epoch.elapsed(), sim_time(run))
                    }
                    Ok(Command::Step) => {
                        interaction.cancel(run, &worker.display)?;
                        running = false;
                        timing.pause(epoch.elapsed());
                        single_step = true;
                        break;
                    }
                    Ok(Command::Reset) => {
                        interaction.cancel(run, &worker.display)?;
                        running = false;
                        run.reset().await.map_err(|e| e.to_string())?;
                        timing = Timing::new(timing.mode);
                        snapshots.reset(run.state().clone());
                        if let Some(viewport) = &mut viewport {
                            viewport.reset(run.generation(), &worker.display)?;
                        }
                        frame_due = true;
                    }
                    Ok(Command::Stop) | Err(TryRecvError::Disconnected) => break 'execution,
                    Err(TryRecvError::Empty) => break,
                }
            }
        }
        if let (Some(viewport), Some(worker)) = (&mut viewport, &desktop) {
            let view_changed = {
                let state = worker
                    .display
                    .lock()
                    .map_err(|_| "desktop state lock poisoned")?;
                state.pending_camera.is_some() || state.pending_scene.is_some()
            };
            if view_changed {
                interaction.cancel(run, &worker.display)?;
            }
            frame_due |= viewport.input(&worker.display)?;
            if interaction.input(run, viewport, &worker.display, !running)? {
                snapshots.refresh(run.state().clone());
                frame_due = true;
            }
        }
        let delay = timing.delay(epoch.elapsed(), sim_time(run));
        if run.boundary() >= requested_steps && delay.is_zero() {
            timing.pause(epoch.elapsed());
            running = false;
            if desktop.is_none() {
                break;
            }
        }
        if (single_step || (running && delay.is_zero())) && run.boundary() < requested_steps {
            if single_step {
                timing.resume(epoch.elapsed(), sim_time(run));
            }
            step(run).await?;
            snapshots.record(run.state().clone());
            frame_due = true;
            if single_step {
                timing.pause(epoch.elapsed());
            }
        }
        if last_renewal.elapsed() >= Duration::from_millis(500) {
            run.watchdog_tick().await.map_err(|e| e.to_string())?;
            last_renewal = Instant::now();
        }
        if let Some(worker) = &desktop {
            let frame = if frame_due && last_frame.elapsed() >= Duration::from_millis(33) {
                let workspace = viewport
                    .as_mut()
                    .ok_or("desktop rendering workspace missing")?;
                let frame = workspace.render(run.state())?;
                last_frame = Instant::now();
                frame_due = false;
                Some(frame)
            } else {
                None
            };
            let mut state = worker
                .display
                .lock()
                .map_err(|_| "desktop state lock poisoned")?;
            if state.robot_name.is_empty() {
                state
                    .robot_name
                    .clone_from(&run.provenance().robot_bundle_identity);
            }
            state.ready = run.authority_state() == AuthorityState::Acquired;
            state.phase = "";
            state.running = running;
            state.boundary = run.boundary();
            state.generation = run.generation();
            state.time_seconds = run.state().time_seconds();
            state.wall_seconds = timing.wall(epoch.elapsed()).as_secs_f64();
            state.speed = timing.speed(epoch.elapsed(), sim_time(run));
            state.pacing = timing.mode;
            #[cfg(test)]
            if state
                .qualification_cuts
                .back()
                .and_then(|cut| cut["boundary"].as_u64())
                != Some(run.boundary())
            {
                let actuations = run.applied_actuation().map(|applied| applied.requested.iter().map(|item| serde_json::json!({
                    "payload": item.payload, "valid_until_ns": item.valid_until_ns,
                    "membership": item.membership.as_ref().map(|member| serde_json::json!({"source": member.producer, "port": member.port, "sequence": member.sequence, "capture_boundary": member.capture_boundary, "capture_time_ns": member.capture_time_ns, "disposition": member.disposition as i32}))
                })).collect::<Vec<_>>()).unwrap_or_default();
                let controls = run
                    .applied_actuation()
                    .map(|applied| applied.native_controls.to_vec())
                    .unwrap_or_default();
                if state.qualification_cuts.len() == 96 {
                    state.qualification_cuts.pop_front();
                }
                state.qualification_cuts.push_back(serde_json::json!({"boundary": run.boundary(), "time_ns": sim_time(run).as_nanos(), "qpos": run.state().qpos(), "qvel": run.state().qvel(), "controls": controls, "actuation": actuations}));
            }
            if let Some(viewport) = &viewport {
                viewport.publish(&mut state);
            }
            if let Some(frame) = frame {
                state.frame = Some(frame);
            }
            if let Some(applied) = run.applied_actuation() {
                state.controls = applied.native_controls.to_vec();
                state.actuation_boundary = Some(applied.boundary);
                state.actuation_products = applied.requested.len();
            } else {
                state.controls.clear();
                state.actuation_boundary = None;
                state.actuation_products = 0;
            }
        }
        let delay = timing.delay(epoch.elapsed(), sim_time(run));
        if !running || !delay.is_zero() {
            // Camera/UI updates and lease renewal remain live while pacing or paused.
            let wait = if running {
                delay.min(Duration::from_millis(33))
            } else {
                Duration::from_millis(33)
            };
            tokio::select! {
                biased;
                _ = async { match &cancel { Some(cancel) => cancel.wait().await, None => std::future::pending().await }} => break,
                _ = interrupt.recv() => {
                    if let Some(worker) = &desktop { worker.display.lock().map_err(|_| "desktop state lock poisoned")?.close_requested = true; }
                    break;
                }
                _ = terminate.recv() => {
                    if let Some(worker) = &desktop { worker.display.lock().map_err(|_| "desktop state lock poisoned")?.close_requested = true; }
                    break;
                }
                command = async { match desktop.as_mut() {
                    Some(worker) => worker.commands.recv().await,
                    None => std::future::pending().await,
                }} => match command { Some(command) => pending = Some(command), None => break },
                _ = tokio::time::sleep(wait) => {}
            }
        }
    }
    if let Some(worker) = &desktop {
        let mut state = worker
            .display
            .lock()
            .map_err(|_| "desktop state lock poisoned")?;
        state.running = false;
        state.wall_seconds = timing.wall(epoch.elapsed()).as_secs_f64();
        state.speed = None;
    }
    run.cancel_drag();
    Ok(snapshots.values)
}

async fn step<T: SimulationTransport>(run: &mut Run<T>) -> Result<(), String> {
    run.step().await.map(|_| ()).map_err(|e| e.to_string())
}

fn sim_time<T: SimulationTransport>(run: &Run<T>) -> Duration {
    Duration::from_secs_f64(run.state().time_seconds())
}

/// Explicit evidence keeps every boundary; ordinary viewing keeps only current state.
struct History<T> {
    complete: bool,
    values: Vec<T>,
}
impl<T> History<T> {
    fn new(complete: bool, initial: T) -> Self {
        Self {
            complete,
            values: vec![initial],
        }
    }
    fn reset(&mut self, initial: T) {
        self.values.clear();
        self.values.push(initial);
    }
    fn refresh(&mut self, state: T) {
        if let Some(current) = self.values.last_mut() {
            *current = state;
        }
    }
    fn record(&mut self, state: T) {
        if !self.complete {
            self.values.clear();
        }
        self.values.push(state);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pose_edit_refreshes_current_evidence_without_an_extra_boundary() {
        let mut evidence = History::new(true, (0, 0));
        evidence.record((1, 0));
        evidence.refresh((1, 7));
        assert_eq!(evidence.values, [(0, 0), (1, 7)]);
        let mut ordinary = History::new(false, (0, 0));
        ordinary.refresh((0, 7));
        assert_eq!(ordinary.values, [(0, 7)]);
    }
    #[test]
    fn ordinary_history_is_constant_and_explicit_evidence_is_complete() {
        let mut ordinary = History::new(false, 0);
        let mut evidence = History::new(true, 0);
        for boundary in 1..=100000 {
            ordinary.record(boundary);
            evidence.record(boundary);
        }
        assert_eq!(ordinary.values, [100000]);
        assert_eq!(evidence.values.len(), 100001);
        assert!(evidence.values.iter().copied().eq(0..=100000));
        evidence.reset(0);
        assert_eq!(evidence.values, [0]);
    }
}
