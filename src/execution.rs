//! One boundary coordinator for finite headless runs and desktop controls.

use crate::{
    authority::{AuthorityClientError, AuthorityState},
    desktop::{Command, Worker},
    native_provider::ComponentProvider,
    remote::{RemoteSceneError, RemoteSceneRun},
};
use phoxal::session::Simulation;
use phoxal_mujoco::{Model, StateSnapshot, Workspace};
use std::{
    sync::mpsc::TryRecvError,
    time::{Duration, Instant},
};

type Run = RemoteSceneRun<Simulation, ComponentProvider>;

pub(super) async fn drive(
    run: &mut Run,
    model: &Model,
    requested_steps: u64,
    desktop: Option<Worker>,
    auto_run: bool,
) -> Result<Vec<StateSnapshot>, String> {
    let mut running = desktop.is_none() || auto_run;
    let mut snapshots = vec![run.state().clone()];
    let mut viewport = if desktop.is_some() {
        Some(Workspace::new(model).map_err(|e| e.to_string())?)
    } else {
        None
    };
    let mut camera = viewport.as_ref().map(Workspace::default_view_camera);
    let mut frame_due = true;
    let mut last_frame = Instant::now();
    let mut last_renewal = Instant::now();
    let mut interrupt = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())
        .map_err(|e| e.to_string())?;
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        .map_err(|e| e.to_string())?;
    loop {
        let stop_signal = tokio::select! {
            biased;
            _ = interrupt.recv() => true,
            _ = terminate.recv() => true,
            _ = std::future::ready(()) => false,
        };
        if stop_signal {
            break;
        }
        let mut single_step = false;
        if let Some(worker) = &desktop {
            if let Some(view) = worker
                .display
                .lock()
                .map_err(|_| "desktop state lock poisoned")?
                .pending_camera
                .take()
            {
                camera = Some(view);
                frame_due = true;
            }
            match worker.commands.try_recv() {
                Ok(Command::Run) => running = true,
                Ok(Command::Pause) => running = false,
                Ok(Command::Step) => {
                    running = false;
                    single_step = true;
                }
                Ok(Command::Reset) => {
                    running = false;
                    run.reset().await.map_err(|e| e.to_string())?;
                    frame_due = true;
                }
                Ok(Command::Stop) | Err(TryRecvError::Disconnected) => break,
                Err(TryRecvError::Empty) => {}
            }
        }
        if run.boundary() >= requested_steps {
            running = false;
            if desktop.is_none() {
                break;
            }
        }
        if (running || single_step) && run.boundary() < requested_steps {
            step(run).await?;
            if run.boundary().is_multiple_of(10) || run.boundary() == requested_steps {
                snapshots.push(run.state().clone());
            }
            frame_due = true;
        }
        if last_renewal.elapsed() >= Duration::from_millis(500) {
            run.watchdog_tick().await.map_err(|e| e.to_string())?;
            last_renewal = Instant::now();
        }
        if let Some(worker) = &desktop {
            let frame =
                if frame_due && (!running || last_frame.elapsed() >= Duration::from_millis(33)) {
                    let workspace = viewport
                        .as_mut()
                        .ok_or("desktop rendering workspace missing")?;
                    workspace
                        .set_qpos(run.state().qpos())
                        .map_err(|e| e.to_string())?;
                    workspace
                        .set_qvel(run.state().qvel())
                        .map_err(|e| e.to_string())?;
                    workspace.forward().map_err(|e| e.to_string())?;
                    let [width, height] = workspace.framebuffer_resolution();
                    let ratio = (1024.0 / width as f64).min(768.0 / height as f64).min(1.0);
                    let size = [
                        (width as f64 * ratio) as usize,
                        (height as f64 * ratio) as usize,
                    ];
                    let frame = workspace
                        .render_viewport(camera.ok_or("viewport camera missing")?, size)
                        .map_err(|e| e.to_string())?;
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
            state.ready = run.authority_state() == AuthorityState::Acquired;
            state.running = running;
            state.boundary = run.boundary();
            state.generation = run.generation();
            state.time_seconds = run.state().time_seconds();
            state.camera = camera;
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
        if !running {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }
    Ok(snapshots)
}

async fn step(run: &mut Run) -> Result<(), String> {
    let mut result = run.step().await.map(|_| ());
    // Retry only through retained-receipt reconciliation. Never rerun a native step.
    for _ in 0..3 {
        if !matches!(
            &result,
            Err(RemoteSceneError::Authority(
                AuthorityClientError::UncertainPhase { .. }
            ))
        ) {
            break;
        }
        result = run.retry_uncertain().await.map(|_| ());
    }
    result.map_err(|e| e.to_string())
}
