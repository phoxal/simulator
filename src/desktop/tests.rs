use super::*;
use std::time::{Duration, Instant};

/// Explicit host qualification of the real worker, supervisor and native owner.
/// Supplies already built artifacts; this test performs no Cargo or acquisition.
#[test]
#[ignore = "requires PHOXAL_QUALIFICATION_BUILD, PHOXAL_QUALIFICATION_SCENE and MuJoCo"]
fn native_worker_pacing_controls_and_cleanup() {
    let options = Options {
        simulation_run: None,
        probe: false,
        scene: std::env::var_os("PHOXAL_QUALIFICATION_SCENE")
            .expect("scene")
            .into(),
        bundle: std::env::var_os("PHOXAL_QUALIFICATION_BUILD")
            .expect("build")
            .into(),
        json: false,
        presentation: crate::config::Presentation::Desktop,
        scope: Some("local".into()),
        connect: None,
        supervisor_id: Some("local".into()),
        run_id: Some("pacing-qualification".into()),
        bound: None,
        auto_run: false,
    };
    let display = Arc::new(Mutex::new(DisplayState::default()));
    let mut owner = start_worker(options.clone(), display.clone()).unwrap();
    wait(&display, |state| state.ready);
    assert_eq!(display.lock().unwrap().pacing, Pacing::Realtime);
    assert_eq!(display.lock().unwrap().wall_seconds, 0.0);
    owner.commands.try_send(Command::Step).unwrap();
    wait(&display, |state| state.boundary == 1);
    assert!(!display.lock().unwrap().running);
    let stepped_wall = display.lock().unwrap().wall_seconds;
    std::thread::sleep(Duration::from_millis(1100));
    assert_eq!(display.lock().unwrap().boundary, 1);
    assert_eq!(display.lock().unwrap().wall_seconds, stepped_wall);
    owner.commands.try_send(Command::Run).unwrap();
    wait(&display, |state| state.boundary >= 101);
    let recent = display.lock().unwrap().speed;
    owner.commands.try_send(Command::Pause).unwrap();
    wait(&display, |state| !state.running);
    {
        let state = display.lock().unwrap();
        let sim = state.time_seconds - 0.01;
        let wall = state.wall_seconds - stepped_wall;
        println!(
            "REALTIME boundary={} sim={sim:.6}s active_wall={wall:.6}s achieved={:.6}x recent={:?}",
            state.boundary,
            sim / wall,
            recent
        );
        // Broad invariant, not a scheduler tolerance: no several-fold fast execution.
        assert!(sim / wall <= 1.1);
    }
    owner
        .commands
        .try_send(Command::SetPacing(Pacing::Fast))
        .unwrap();
    wait(&display, |state| state.pacing == Pacing::Fast);
    let fast_start = {
        let state = display.lock().unwrap();
        (state.boundary, state.time_seconds, state.wall_seconds)
    };
    owner.commands.try_send(Command::Run).unwrap();
    wait(&display, |state| state.boundary >= fast_start.0 + 100);
    owner.commands.try_send(Command::Pause).unwrap();
    wait(&display, |state| !state.running);
    {
        let state = display.lock().unwrap();
        println!(
            "FAST boundary={} sim={:.6}s active_wall={:.6}s achieved={:.6}x",
            state.boundary,
            state.time_seconds - fast_start.1,
            state.wall_seconds - fast_start.2,
            (state.time_seconds - fast_start.1) / (state.wall_seconds - fast_start.2)
        );
    }
    owner
        .commands
        .try_send(Command::SetPacing(Pacing::Realtime))
        .unwrap();
    wait(&display, |state| state.pacing == Pacing::Realtime);
    let before = display.lock().unwrap().boundary;
    owner.commands.try_send(Command::Step).unwrap();
    wait(&display, |state| state.boundary == before + 1);
    owner.commands.try_send(Command::Reset).unwrap();
    wait(&display, |state| state.boundary == 0);
    {
        let state = display.lock().unwrap();
        assert_eq!(state.time_seconds, 0.0);
        assert_eq!(state.wall_seconds, 0.0);
        assert_eq!(state.speed, None);
        assert!(!state.running);
    }
    owner.commands.try_send(Command::Run).unwrap();
    wait(&display, |state| state.boundary >= 5);
    owner.commands.try_send(Command::Pause).unwrap();
    wait(&display, |state| !state.running);
    assert!(display.lock().unwrap().wall_seconds > 0.0);
    owner.commands.try_send(Command::Stop).unwrap();
    wait(&display, |state| state.finished);
    assert!(owner.thread.take().unwrap().join().unwrap().is_ok());
    assert!(!display.lock().unwrap().ready);
    assert_eq!(display.lock().unwrap().speed, None);
    let mut options = options;
    options.run_id = Some("pacing-qualification-restart".into());
    options.auto_run = false;
    *display.lock().unwrap() = DisplayState::default();
    owner = start_worker(options, display.clone()).unwrap();
    wait(&display, |state| state.ready);
    {
        let state = display.lock().unwrap();
        assert_eq!(state.pacing, Pacing::Realtime);
        assert_eq!(state.wall_seconds, 0.0);
        assert_eq!(state.boundary, 0);
    }
    owner.commands.try_send(Command::Run).unwrap();
    wait(&display, |state| state.boundary >= 1);
    let (disconnected, _) = mpsc::channel(1);
    owner.commands = disconnected;
    wait(&display, |state| state.finished);
    assert!(owner.thread.take().unwrap().join().unwrap().is_ok());
    println!(
        "CONTROL pause-stable, step, run, both pacing acknowledgements, reset, stop, restart and channel-close joined cleanup passed"
    );
}

fn wait(display: &Arc<Mutex<DisplayState>>, predicate: impl Fn(&DisplayState) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        {
            let state = display.lock().unwrap();
            assert!(state.error.is_none(), "worker error: {:?}", state.error);
            if predicate(&state) {
                return;
            }
        }
        assert!(Instant::now() < deadline, "worker acknowledgement timeout");
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// Generic external stepped-source qualification; opaque input belongs to its fixture.
/// This driver knows native controls and commands, not any participant implementation.
#[test]
#[ignore = "requires native bundle, scene, external input fixture and case JSON"]
fn native_external_source_pause_trace() {
    let input = std::path::PathBuf::from(
        std::env::var_os("PHOXAL_QUALIFICATION_INPUT").expect("input file"),
    );
    let cases: Vec<serde_json::Value> =
        serde_json::from_str(&std::env::var("PHOXAL_QUALIFICATION_CASES").expect("case JSON"))
            .unwrap();
    let directory = std::path::PathBuf::from(
        std::env::var_os("PHOXAL_QUALIFICATION_TRACE_DIR").expect("trace directory"),
    );
    std::fs::create_dir_all(&directory).unwrap();
    let write_input = |value: &str| {
        let temporary = input.with_extension("next");
        std::fs::write(&temporary, value).unwrap();
        std::fs::rename(&temporary, &input).unwrap();
    };
    let neutral = std::env::var("PHOXAL_QUALIFICATION_NEUTRAL").unwrap();
    let engaged = std::env::var("PHOXAL_QUALIFICATION_ENGAGED").unwrap();
    let active = std::env::var("PHOXAL_QUALIFICATION_ACTIVE").unwrap();
    for phase in 0..=1 {
        for (case_index, case) in cases.iter().enumerate() {
            write_input(&neutral);
            let options = Options {
                simulation_run: None,
                probe: false,
                scene: std::env::var_os("PHOXAL_QUALIFICATION_SCENE")
                    .unwrap()
                    .into(),
                bundle: std::env::var_os("PHOXAL_QUALIFICATION_BUILD")
                    .unwrap()
                    .into(),
                json: false,
                presentation: crate::config::Presentation::Desktop,
                scope: Some("local".into()),
                connect: None,
                supervisor_id: Some("local".into()),
                run_id: Some(format!("external-pause-{phase}-{case_index}")),
                bound: None,
                auto_run: false,
            };
            let display = Arc::new(Mutex::new(DisplayState::default()));
            let mut owner = start_worker(options, display.clone()).unwrap();
            wait(&display, |state| state.ready);
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let advance = |target: u64| {
                    while display.lock().unwrap().boundary < target {
                        let next = display.lock().unwrap().boundary + 1;
                        owner.commands.try_send(Command::Step).unwrap();
                        wait(&display, |state| state.boundary == next);
                    }
                };
                advance(2);
                write_input(&engaged);
                advance(10);
                write_input(&active);
                advance(20);
                if display
                    .lock()
                    .unwrap()
                    .controls
                    .iter()
                    .all(|control| *control == 0.0)
                {
                    let cuts = display
                        .lock()
                        .unwrap()
                        .qualification_cuts
                        .iter()
                        .cloned()
                        .collect::<Vec<_>>();
                    let report = serde_json::json!({"phase": phase, "case": case, "failure": "fixture did not achieve nonzero authority; pause coverage not reached", "cuts": cuts});
                    std::fs::write(
                        directory.join(format!(
                            "startup-failure-phase-{phase}-case-{case_index}.json"
                        )),
                        serde_json::to_vec_pretty(&report).unwrap(),
                    )
                    .unwrap();
                }
                assert!(
                    display
                        .lock()
                        .unwrap()
                        .controls
                        .iter()
                        .any(|control| *control != 0.0),
                    "fixture did not achieve nonzero native authority"
                );
                owner.commands.try_send(Command::Run).unwrap();
                wait(&display, |state| state.running && state.boundary >= 30);
                owner.commands.try_send(Command::Pause).unwrap();
                wait(&display, |state| !state.running);
                let paused = display.lock().unwrap().boundary;
                let boundary = paused.max(40).div_ceil(2) * 2 + phase;
                advance(boundary);
                let before = display
                    .lock()
                    .unwrap()
                    .qualification_cuts
                    .back()
                    .unwrap()
                    .clone();
                assert!(
                    before["controls"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .any(|value| value.as_f64().unwrap() != 0.0)
                );
                write_input(case["input"].as_str().unwrap());
                // Deliberate paused dwell exceeds the logical lease in wall time.
                // No producer/consumer invocation occurs here, so it cannot bias ordering.
                let paused_at = Instant::now();
                std::thread::sleep(Duration::from_millis(150));
                let paused_wall_ns = paused_at.elapsed().as_nanos();
                // No physics or runtime advancement is induced by changing OS fixture input.
                assert_eq!(display.lock().unwrap().boundary, boundary);
                assert_eq!(
                    display.lock().unwrap().qualification_cuts.back().unwrap(),
                    &before
                );
                owner.commands.try_send(Command::Step).unwrap();
                wait(&display, |state| state.boundary == boundary + 1);
                assert!(!display.lock().unwrap().running);
                let first = display
                    .lock()
                    .unwrap()
                    .qualification_cuts
                    .back()
                    .unwrap()
                    .clone();
                assert_eq!(
                    first["controls"], before["controls"],
                    "normal next-boundary admission should retain the earlier command on this first Step"
                );
                owner.commands.try_send(Command::Run).unwrap();
                wait(&display, |state| {
                    state.running && state.boundary >= boundary + 7
                });
                owner.commands.try_send(Command::Pause).unwrap();
                wait(&display, |state| !state.running);
                let after = display
                    .lock()
                    .unwrap()
                    .qualification_cuts
                    .back()
                    .unwrap()
                    .clone();
                let expect_inactive = case["expect_inactive"].as_bool().unwrap();
                let controls = after["controls"].as_array().unwrap();
                assert_eq!(
                    controls.iter().all(|value| value.as_f64().unwrap() == 0.0),
                    expect_inactive
                );
                let cuts = display
                    .lock()
                    .unwrap()
                    .qualification_cuts
                    .iter()
                    .cloned()
                    .collect::<Vec<_>>();
                let first_inactive = cuts
                    .iter()
                    .find(|cut| {
                        cut["boundary"].as_u64().unwrap() > boundary
                            && cut["controls"]
                                .as_array()
                                .unwrap()
                                .iter()
                                .all(|value| value.as_f64().unwrap() == 0.0)
                    })
                    .and_then(|cut| cut["boundary"].as_u64());
                if expect_inactive {
                    assert!(
                        first_inactive.is_some_and(|stopped| stopped <= boundary + 4),
                        "withdrawal exceeded existing sampling bound"
                    );
                    // Pure freeze preserves the prior admitted nondue command.
                    // The following normal source/consumer cuts must still stop it.
                    if phase == 1 {
                        assert!(
                            first["controls"]
                                .as_array()
                                .unwrap()
                                .iter()
                                .any(|value| value.as_f64().unwrap() != 0.0)
                        );
                    }
                }
                let mut report = serde_json::json!({"pause_wall_ns": paused_wall_ns, "case": case, "prepare_boundary": boundary, "prepare_time_ns": boundary * 10_000_000, "next_due": phase == 0,
                "before": before, "first_step": first, "after_run": after, "first_inactive": first_inactive, "cuts": cuts,
                "first_resume_reuses_admitted_control": first["controls"] == before["controls"]});
                std::fs::write(
                    directory.join(format!("phase-{phase}-case-{case_index}.json")),
                    serde_json::to_vec_pretty(&report).unwrap(),
                )
                .unwrap();
                println!(
                    "EXTERNAL_PAUSE phase={phase} case={case_index} prepare={boundary} logical_ns={} first_inactive={first_inactive:?} first_controls={} after_controls={}",
                    boundary * 10_000_000,
                    first["controls"],
                    after["controls"]
                );
                if case["reengage"].as_bool().unwrap_or(false) {
                    write_input(&neutral);
                    let released_boundary = display.lock().unwrap().boundary + 4;
                    advance(released_boundary);
                    assert!(
                        display
                            .lock()
                            .unwrap()
                            .controls
                            .iter()
                            .all(|control| *control == 0.0)
                    );
                    write_input(&engaged);
                    let armed_boundary = display.lock().unwrap().boundary + 10;
                    advance(armed_boundary);
                    write_input(&active);
                    let moving_boundary = display.lock().unwrap().boundary + 6;
                    advance(moving_boundary);
                    assert!(
                        display
                            .lock()
                            .unwrap()
                            .controls
                            .iter()
                            .any(|control| *control != 0.0),
                        "fresh normal engagement did not restore authority"
                    );
                    report["fresh_reengagement"] = display
                        .lock()
                        .unwrap()
                        .qualification_cuts
                        .back()
                        .unwrap()
                        .clone();
                    std::fs::write(
                        directory.join(format!("phase-{phase}-case-{case_index}.json")),
                        serde_json::to_vec_pretty(&report).unwrap(),
                    )
                    .unwrap();
                }
                write_input(&neutral);
                let cleanup_boundary = display.lock().unwrap().boundary + 6;
                advance(cleanup_boundary);
                assert!(
                    display
                        .lock()
                        .unwrap()
                        .controls
                        .iter()
                        .all(|value| *value == 0.0)
                );
            }));
            let _ = owner.commands.try_send(Command::Stop);
            assert!(owner.thread.take().unwrap().join().unwrap().is_ok());
            assert!(display.lock().unwrap().finished);
            println!("EXTERNAL_CLEANUP phase={phase} case={case_index} worker joined");
            if let Err(payload) = outcome {
                std::panic::resume_unwind(payload);
            }
        }
    }
}

#[test]
fn desktop_layout_retains_a_dominant_unclipped_viewport() {
    for [width, height] in [[480.0, 420.0], [720.0, 560.0], [1200.0, 820.0]] {
        let ctx = egui::Context::default();
        let (sender, _) = mpsc::channel(8);
        let mut desktop = Desktop {
            control: Arc::new(Mutex::new(Control {
                commands: sender,
                thread: None,
            })),
            display: Arc::new(Mutex::new(DisplayState {
                ready: true,
                ..Default::default()
            })),
            options: None,
            build_path: String::new(),
            scene_path: String::new(),
            restart: 0,
            texture: Some(ctx.load_texture(
                "layout-fixture",
                egui::ColorImage::filled([64, 48], egui::Color32::BLACK),
                egui::TextureOptions::LINEAR,
            )),
            message: None,
            view_camera: None,
            gesture: None,
            next_gesture: 0,
            pointer_cut: None,
            last_viewport: None,
        };
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, height));
        for _ in 0..3 {
            let mut output = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(screen),
                    ..Default::default()
                },
                |ctx| {
                    desktop.draw(ctx);
                },
            );
            // This arithmetic/layout test has no GPU consumer.
            output.textures_delta.clear();
        }
        let viewport = desktop.last_viewport.unwrap();
        assert!(screen.contains_rect(viewport), "{screen:?} vs {viewport:?}");
        assert!(viewport.height() > 120.0, "{viewport:?}");
        assert!(viewport.width() > width * 0.6, "{viewport:?}");
    }
}

#[test]
#[ignore = "requires PHOXAL_QUALIFICATION_BUILD, PHOXAL_QUALIFICATION_SCENE and MuJoCo"]
fn native_worker_scene_selection_fences_and_cleanup() {
    let mut options = Options {
        simulation_run: None,
        probe: false,
        scene: std::env::var_os("PHOXAL_QUALIFICATION_SCENE")
            .expect("scene")
            .into(),
        bundle: std::env::var_os("PHOXAL_QUALIFICATION_BUILD")
            .expect("build")
            .into(),
        json: false,
        presentation: crate::config::Presentation::Desktop,
        scope: Some("local".into()),
        connect: None,
        supervisor_id: Some("local".into()),
        run_id: Some("scene-selection-proof".into()),
        bound: None,
        auto_run: false,
    };
    let display = Arc::new(Mutex::new(DisplayState::default()));
    let mut owner = start_worker(options.clone(), display.clone()).unwrap();
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        wait(&display, |state| state.ready && state.frame.is_some());
        let (before, body, original) = {
            let mut state = display.lock().unwrap();
            let frame = state.frame.take().unwrap();
            save_native_frame("before", &frame.image);
            let original = frame.view.clone();
            state.presented = Some(frame.view);
            let body = state
                .bodies
                .iter()
                .find(|body| body.mobility.label() == "Free joint")
                .unwrap()
                .id;
            queue_scene(&mut state, scene::SceneAction::SelectBody(body));
            (
                state.presented.as_ref().unwrap().snapshot.clone(),
                body,
                original,
            )
        };
        wait(&display, |state| {
            state.selection.as_ref().is_some_and(|hit| hit.body == body) && state.frame.is_some()
        });
        {
            let mut state = display.lock().unwrap();
            let frame = state.frame.take().unwrap();
            save_native_frame("selected", &frame.image);
            assert_eq!(frame.view.snapshot, before);
            state.presented = Some(frame.view);
            queue_scene(&mut state, scene::SceneAction::Focus);
        }
        wait(&display, |state| {
            state
                .scene_result
                .as_ref()
                .is_some_and(|result| result.camera.is_some())
        });
        let focused = display
            .lock()
            .unwrap()
            .scene_result
            .take()
            .unwrap()
            .camera
            .unwrap();
        wait(&display, |state| {
            state
                .frame
                .as_ref()
                .is_some_and(|frame| frame.view.camera == focused)
        });
        {
            let mut state = display.lock().unwrap();
            let frame = state.frame.take().unwrap();
            save_native_frame("focused", &frame.image);
            state.presented = Some(frame.view);
            queue_scene(&mut state, scene::SceneAction::Pick([0.5, 0.5]));
        }
        wait(&display, |state| {
            state.scene_result.is_some()
                && state
                    .selection
                    .as_ref()
                    .is_some_and(|hit| hit.geom.is_some())
        });
        {
            let mut state = display.lock().unwrap();
            assert!(!state.scene_result.take().unwrap().stale);
            println!("NATIVE_SCENE_SELECTION {:?}", state.selection);
            assert!(state.selection.is_some());
            assert_eq!(state.boundary, 0);
            queue_scene(&mut state, scene::SceneAction::Clear);
        }
        wait(&display, |state| {
            state.selection.is_none() && state.scene_result.is_some()
        });
        owner.commands.try_send(Command::Step).unwrap();
        wait(&display, |state| state.boundary == 1);
        owner.commands.try_send(Command::Reset).unwrap();
        wait(&display, |state| {
            state.boundary == 0 && state.generation > 0
        });
        {
            let mut state = display.lock().unwrap();
            assert!(state.selection.is_none());
            state.presented = Some(original.clone());
            state.pending_scene = Some(scene::SceneRequest {
                frame: original.identity.clone(),
                action: scene::SceneAction::SelectBody(body),
            });
        }
        wait(&display, |state| {
            state
                .scene_result
                .as_ref()
                .is_some_and(|result| result.stale)
        });
        assert!(display.lock().unwrap().selection.is_none());
    }));
    let old_execution = display
        .lock()
        .unwrap()
        .presented
        .as_ref()
        .map(|view| view.identity.epoch.execution.clone());
    let _ = owner.commands.try_send(Command::Stop);
    assert!(owner.thread.take().unwrap().join().unwrap().is_ok());
    assert!(display.lock().unwrap().presented.is_none());
    assert!(display.lock().unwrap().selection.is_none());
    if let Err(payload) = outcome {
        std::panic::resume_unwind(payload);
    }
    let original_epoch = display.lock().unwrap().generation;
    options.run_id = Some("scene-selection-proof-restart".into());
    *display.lock().unwrap() = DisplayState::default();
    let mut restarted = start_worker(options, display.clone()).unwrap();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        wait(&display, |state| state.ready && state.frame.is_some());
        let new_execution = display
            .lock()
            .unwrap()
            .frame
            .as_ref()
            .unwrap()
            .view
            .identity
            .epoch
            .execution
            .clone();
        assert_ne!(Some(new_execution), old_execution);
        assert!(original_epoch > 0);
        assert!(display.lock().unwrap().selection.is_none());
    }));
    let _ = restarted.commands.try_send(Command::Stop);
    assert!(restarted.thread.take().unwrap().join().unwrap().is_ok());
    if let Err(payload) = result {
        std::panic::resume_unwind(payload);
    }
}
fn save_native_frame(name: &str, image: &crate::mujoco::RenderedCamera) {
    if let Some(directory) = std::env::var_os("PHOXAL_QUALIFICATION_IMAGES") {
        let directory = std::path::PathBuf::from(directory);
        std::fs::create_dir_all(&directory).unwrap();
        let [width, height] = image.resolution();
        let mut bytes = format!("P6\n{width} {height}\n255\n").into_bytes();
        bytes.extend_from_slice(image.rgb());
        std::fs::write(directory.join(format!("{name}.ppm")), bytes).unwrap();
    }
}

#[test]
#[ignore = "requires actual common rover build, scene and MuJoCo"]
fn native_worker_physical_drag_reposition_and_cleanup() {
    let options = Options {
        simulation_run: None,
        probe: false,
        scene: std::env::var_os("PHOXAL_QUALIFICATION_SCENE")
            .expect("scene")
            .into(),
        bundle: std::env::var_os("PHOXAL_QUALIFICATION_BUILD")
            .expect("build")
            .into(),
        json: false,
        presentation: crate::config::Presentation::Desktop,
        scope: Some("local".into()),
        connect: None,
        supervisor_id: Some("local".into()),
        run_id: Some("physical-interaction-proof".into()),
        bound: None,
        auto_run: false,
    };
    let display = Arc::new(Mutex::new(DisplayState::default()));
    let mut owner = start_worker(options.clone(), display.clone()).unwrap();
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        wait(&display, |s| s.ready && s.frame.is_some());
        let original = {
            let mut s = display.lock().unwrap();
            let frame = s.frame.take().unwrap();
            save_native_frame("drag-original", &frame.image);
            let original = frame.view.snapshot.clone();
            s.presented = Some(frame.view);
            let root = s
                .bodies
                .iter()
                .find(|b| b.mobility.label() == "Free joint")
                .unwrap()
                .id;
            queue_scene(&mut s, scene::SceneAction::SelectBody(root));
            original
        };
        wait(&display, |s| s.selection.is_some() && s.frame.is_some());
        {
            let mut s = display.lock().unwrap();
            s.presented = Some(s.frame.take().unwrap().view);
            queue_scene(&mut s, scene::SceneAction::Focus);
        }
        wait(&display, |s| {
            s.scene_result.as_ref().is_some_and(|r| r.camera.is_some())
        });
        let camera = display
            .lock()
            .unwrap()
            .scene_result
            .take()
            .unwrap()
            .camera
            .unwrap();
        wait(&display, |s| {
            s.frame.as_ref().is_some_and(|f| f.view.camera == camera)
        });
        let original_view = {
            let mut s = display.lock().unwrap();
            s.presented = Some(s.frame.take().unwrap().view);
            let view = s.presented.as_ref().unwrap().clone();
            s.drag_mailbox.begin = Some(scene::DragBegin {
                id: 1,
                frame: view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
            s.drag_mailbox.update = Some(scene::DragUpdate {
                id: 1,
                epoch: view.identity.epoch.clone(),
                delta: [0.08, 0.0],
                received: Instant::now(),
            });
            view
        };
        wait(&display, |s| {
            s.dragging
                && s.frame
                    .as_ref()
                    .is_some_and(|f| f.view.snapshot.qpos() != original.qpos())
        });
        let edited = {
            let mut s = display.lock().unwrap();
            let f = s.frame.take().unwrap();
            save_native_frame("drag-paused-edited", &f.image);
            assert_eq!(f.view.snapshot.boundary(), 0);
            assert_eq!(f.view.snapshot.time_seconds(), 0.0);
            assert!(f.view.snapshot.qvel()[..6].iter().all(|v| *v == 0.0));
            let edited = f.view.snapshot.clone();
            s.presented = Some(f.view);
            s.drag_mailbox.end(1);
            edited
        };
        wait(&display, |s| !s.dragging);
        owner.commands.try_send(Command::Step).unwrap();
        wait(&display, |s| {
            s.boundary == 1
                && s.frame
                    .as_ref()
                    .is_some_and(|f| f.view.snapshot.boundary() == 1)
        });
        {
            let s = display.lock().unwrap();
            let stepped = &s.frame.as_ref().unwrap().view.snapshot;
            assert!((stepped.qpos()[0] - edited.qpos()[0]).abs() < 0.01);
            println!(
                "NATIVE_PAUSED_EDIT original={:?} edited={:?} stepped={:?} boundary=1 time=.01",
                original.qpos(),
                edited.qpos(),
                stepped.qpos()
            );
        }
        owner.commands.try_send(Command::Run).unwrap();
        wait(&display, |s| s.boundary >= 5);
        owner.commands.try_send(Command::Pause).unwrap();
        wait(&display, |s| !s.running);
        let boundary = display.lock().unwrap().boundary;
        wait(&display, |s| {
            s.frame
                .as_ref()
                .is_some_and(|f| f.view.snapshot.boundary() == boundary)
        });
        {
            let mut s = display.lock().unwrap();
            s.presented = Some(s.frame.take().unwrap().view);
            let view = s.presented.as_ref().unwrap().clone();
            s.drag_mailbox.begin = Some(scene::DragBegin {
                id: 2,
                frame: view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
        }
        wait(&display, |s| s.dragging);
        // Run cancels a paused pose gesture; a fresh gesture is physically applied.
        owner.commands.try_send(Command::Run).unwrap();
        wait(&display, |s| s.running && !s.dragging && s.frame.is_some());
        {
            let mut s = display.lock().unwrap();
            s.presented = Some(s.frame.take().unwrap().view);
            let view = s.presented.as_ref().unwrap().clone();
            s.drag_mailbox.begin = Some(scene::DragBegin {
                id: 3,
                frame: view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
            s.drag_mailbox.update = Some(scene::DragUpdate {
                id: 3,
                epoch: view.identity.epoch.clone(),
                delta: [0.08, 0.0],
                received: Instant::now(),
            });
        }
        wait(&display, |s| s.dragging);
        let from = display.lock().unwrap().boundary;
        for _ in 0..5 {
            std::thread::sleep(Duration::from_millis(80));
            let mut s = display.lock().unwrap();
            assert!(s.dragging);
            s.drag_mailbox.update = Some(scene::DragUpdate {
                id: 3,
                epoch: s.presented.as_ref().unwrap().identity.epoch.clone(),
                delta: [0.08, 0.0],
                received: Instant::now(),
            });
        }
        wait(&display, |s| s.boundary >= from + 5);
        assert!(display.lock().unwrap().dragging);
        println!(
            "NATIVE_STATIONARY_HOLD renewed unchanged target across newer rendering for >250ms"
        );
        // Missing held renewal expires the GUI force; momentum can keep coasting.
        wait(&display, |s| !s.dragging);
        {
            let mut s = display.lock().unwrap();
            s.drag_mailbox.update = Some(scene::DragUpdate {
                id: 3,
                epoch: s.presented.as_ref().unwrap().identity.epoch.clone(),
                delta: [0.08, 0.0],
                received: Instant::now(),
            });
        }
        wait(&display, |s| s.drag_mailbox.update.is_none());
        assert!(!display.lock().unwrap().dragging);
        owner.commands.try_send(Command::Pause).unwrap();
        wait(&display, |s| !s.running && s.frame.is_some());
        {
            let s = display.lock().unwrap();
            let snapshot = &s.frame.as_ref().unwrap().view.snapshot;
            println!(
                "NATIVE_RUNNING_PUSH boundary={} qpos={:?} qvel={:?} drag_timeout=true",
                s.boundary,
                snapshot.qpos(),
                snapshot.qvel()
            );
            assert_ne!(snapshot.qpos(), edited.qpos());
            save_native_frame("drag-after-force", &s.frame.as_ref().unwrap().image);
        }
        let old_generation = display.lock().unwrap().generation;
        owner.commands.try_send(Command::Reset).unwrap();
        wait(&display, |s| {
            s.generation > old_generation
                && s.boundary == 0
                && s.frame
                    .as_ref()
                    .is_some_and(|f| f.view.snapshot.boundary() == 0)
        });
        {
            let mut s = display.lock().unwrap();
            assert_eq!(s.frame.as_ref().unwrap().view.snapshot, original);
            save_native_frame("drag-reset", &s.frame.as_ref().unwrap().image);
            s.presented = Some(original_view.clone());
            s.drag_mailbox.begin = Some(scene::DragBegin {
                id: 4,
                frame: original_view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
        }
        wait(&display, |s| s.interaction_error.is_some());
        assert!(!display.lock().unwrap().dragging);
    }));
    let stale_view = display.lock().unwrap().presented.clone();
    let _ = owner.commands.try_send(Command::Stop);
    assert!(owner.thread.take().unwrap().join().unwrap().is_ok());
    assert!(!display.lock().unwrap().dragging);
    if let Err(payload) = outcome {
        std::panic::resume_unwind(payload);
    }
    *display.lock().unwrap() = DisplayState::default();
    let mut restarted = start_worker(options, display.clone()).unwrap();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        wait(&display, |s| s.ready && s.frame.is_some());
        if let Some(view) = stale_view {
            let mut s = display.lock().unwrap();
            s.presented = Some(view.clone());
            s.drag_mailbox.begin = Some(scene::DragBegin {
                id: 5,
                frame: view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
            drop(s);
            wait(&display, |s| s.interaction_error.is_some());
            assert!(!display.lock().unwrap().dragging);
        }
        {
            let mut s = display.lock().unwrap();
            let root = s
                .bodies
                .iter()
                .find(|b| b.mobility.label() == "Free joint")
                .unwrap()
                .id;
            s.presented = Some(s.frame.take().unwrap().view);
            queue_scene(&mut s, scene::SceneAction::SelectBody(root));
        }
        wait(&display, |s| s.selection.is_some() && s.frame.is_some());
        {
            let mut s = display.lock().unwrap();
            s.presented = Some(s.frame.take().unwrap().view);
            queue_scene(&mut s, scene::SceneAction::Focus);
        }
        wait(&display, |s| {
            s.scene_result.as_ref().is_some_and(|r| r.camera.is_some())
        });
        let camera = display
            .lock()
            .unwrap()
            .scene_result
            .take()
            .unwrap()
            .camera
            .unwrap();
        wait(&display, |s| {
            s.frame.as_ref().is_some_and(|f| f.view.camera == camera)
        });
        {
            let mut s = display.lock().unwrap();
            s.presented = Some(s.frame.take().unwrap().view);
            let view = s.presented.as_ref().unwrap().clone();
            s.drag_mailbox.begin = Some(scene::DragBegin {
                id: 6,
                frame: view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
            s.drag_mailbox.update = Some(scene::DragUpdate {
                id: 6,
                epoch: view.identity.epoch.clone(),
                delta: [0.08, 0.0],
                received: Instant::now(),
            });
        }
        wait(&display, |s| {
            s.dragging
                && s.frame
                    .as_ref()
                    .is_some_and(|f| f.view.snapshot.qpos()[0].abs() > 0.01)
        });
        {
            let s = display.lock().unwrap();
            let frame = s.frame.as_ref().unwrap();
            println!(
                "NATIVE_PAUSED_CLOSE {}",
                serde_json::json!({"boundary":0,"position": &frame.view.snapshot.qpos()[..3]})
            );
        }
        let (disconnected, _) = mpsc::channel(1);
        restarted.commands = disconnected;
        wait(&display, |s| s.finished);
        assert!(!display.lock().unwrap().dragging);
    }));
    let _ = restarted.commands.try_send(Command::Stop);
    assert!(restarted.thread.take().unwrap().join().unwrap().is_ok());
    if let Err(payload) = result {
        std::panic::resume_unwind(payload);
    }
}

#[test]
fn expanded_inspector_and_diagnostics_preserve_viewport_across_resizing() {
    fn frame(
        ctx: &egui::Context,
        desktop: &mut Desktop,
        size: [f32; 2],
        events: Vec<egui::Event>,
        time: f64,
    ) -> egui::FullOutput {
        let mut output = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(size[0], size[1]),
                )),
                events,
                time: Some(time),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        output.textures_delta.clear();
        output
    }
    fn header(output: &egui::FullOutput, name: &str) -> egui::Pos2 {
        output
            .shapes
            .iter()
            .find_map(|shape| {
                if let egui::epaint::Shape::Text(text) = &shape.shape
                    && text.galley.text() == name
                {
                    Some(text.pos + egui::vec2(3.0, 3.0))
                } else {
                    None
                }
            })
            .unwrap_or_else(|| panic!("header not visible: {name}"))
    }
    for diagnostics_open in [false, true] {
        let ctx = egui::Context::default();
        let (sender, _) = mpsc::channel(8);
        let mut desktop = Desktop {
            control: Arc::new(Mutex::new(Control {
                commands: sender,
                thread: None,
            })),
            display: Arc::new(Mutex::new(DisplayState {
                ready: true,
                robot_name: "robot-rover".into(),
                bodies: (0..32)
                    .map(|id| NativeBody {
                        id,
                        name: format!("robot-rover__body_{id}__long_native_identity"),
                        mobility: BodyMobility::FreeJoint,
                    })
                    .collect::<Vec<_>>()
                    .into(),
                selection: Some(NativeSelection {
                    body: 1,
                    geom: Some(1),
                    point: [0.1, 0.2, 0.3],
                }),
                ..Default::default()
            })),
            options: None,
            build_path: String::new(),
            scene_path: String::new(),
            restart: 0,
            texture: Some(ctx.load_texture(
                "expanded-layout",
                egui::ColorImage::filled([64, 48], egui::Color32::BLACK),
                egui::TextureOptions::LINEAR,
            )),
            message: Some(
                "A diagnostic remains accessible without consuming the viewport.".repeat(4),
            ),
            view_camera: None,
            gesture: None,
            next_gesture: 0,
            pointer_cut: None,
            last_viewport: None,
        };
        let mut time = 0.0;
        let mut scene_opened = false;
        let mut diagnostics_clicked = false;
        for size in [
            [1200.0, 820.0],
            [700.0, 552.0],
            [480.0, 420.0],
            [1200.0, 820.0],
            [700.0, 552.0],
        ] {
            time += 1.0;
            let mut output = frame(&ctx, &mut desktop, size, Vec::new(), time);
            if size[0] < 720.0 {
                // Drive actual header clicks, rather than testing only a
                // default-collapsed region or an artificial size formula.
                if !scene_opened {
                    let point = header(&output, "Scene and selected body");
                    time += 0.1;
                    frame(
                        &ctx,
                        &mut desktop,
                        size,
                        vec![
                            egui::Event::PointerMoved(point),
                            egui::Event::PointerButton {
                                pos: point,
                                button: egui::PointerButton::Primary,
                                pressed: true,
                                modifiers: Default::default(),
                            },
                        ],
                        time,
                    );
                    time += 0.1;
                    frame(
                        &ctx,
                        &mut desktop,
                        size,
                        vec![egui::Event::PointerButton {
                            pos: point,
                            button: egui::PointerButton::Primary,
                            pressed: false,
                            modifiers: Default::default(),
                        }],
                        time,
                    );
                    time += 1.0;
                    output = frame(&ctx, &mut desktop, size, Vec::new(), time);
                    assert!(
                        output.shapes.iter().any(|shape| matches!(&shape.shape,
                        egui::epaint::Shape::Text(text) if text.galley.text() == "Scene")),
                        "inspector did not expand"
                    );
                    scene_opened = true;
                }
                if diagnostics_open && !diagnostics_clicked {
                    let point = header(&output, "Diagnostics");
                    time += 0.1;
                    frame(
                        &ctx,
                        &mut desktop,
                        size,
                        vec![
                            egui::Event::PointerMoved(point),
                            egui::Event::PointerButton {
                                pos: point,
                                button: egui::PointerButton::Primary,
                                pressed: true,
                                modifiers: Default::default(),
                            },
                        ],
                        time,
                    );
                    time += 0.1;
                    frame(
                        &ctx,
                        &mut desktop,
                        size,
                        vec![egui::Event::PointerButton {
                            pos: point,
                            button: egui::PointerButton::Primary,
                            pressed: false,
                            modifiers: Default::default(),
                        }],
                        time,
                    );
                    time += 1.0;
                    frame(&ctx, &mut desktop, size, Vec::new(), time);
                    diagnostics_clicked = true;
                }
            }
            let viewport = desktop.last_viewport.unwrap();
            let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(size[0], size[1]));
            assert!(
                screen.contains_rect(viewport),
                "screen {screen:?}, viewport {viewport:?}"
            );
            assert!(
                viewport.height() >= 120.0,
                "expanded {size:?} diagnostics {diagnostics_open}: {viewport:?}"
            );
            assert!(viewport.width() >= size[0] * 0.6, "{viewport:?}");
            output.textures_delta.clear();
        }
    }
}
