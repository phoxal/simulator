use super::*;
use std::time::{Duration, Instant};

#[test]
fn graphics_failure_retains_the_worker_cleanup_outcome() {
    assert_eq!(desktop_outcome(Ok(()), Ok(())), Ok(()));
    assert_eq!(
        desktop_outcome::<()>(Ok(()), Err("remote release unconfirmed".into())),
        Err("remote release unconfirmed".into())
    );
    let graphics = desktop_outcome(Err("display unavailable".into()), Ok(())).unwrap_err();
    assert!(graphics.contains("display unavailable"));
    assert!(graphics.contains("headless"));
    assert!(!graphics.contains("setup"));
    let both = desktop_outcome::<()>(
        Err("display unavailable".into()),
        Err("remote release unconfirmed".into()),
    )
    .unwrap_err();
    assert!(both.starts_with(&graphics));
    assert!(both.contains("remote release unconfirmed"));
}

#[test]
#[ignore = "requires PHOXAL_QUALIFICATION_FIXTURES with admitted build copies and MuJoCo"]
fn native_worker_qa_readiness_cancel_and_failure_preserve_cleanup_and_diagnostics() {
    let fixtures = std::path::PathBuf::from(
        std::env::var_os("PHOXAL_QUALIFICATION_FIXTURES")
            .expect("explicit qualification fixture directory"),
    );
    for (build, held) in [("held-build", true), ("failure-build", false)] {
        let options = Options {
            scene: fixtures.join("scene.xml"),
            bundle: fixtures.join(build),
            simulation_run: None,
            probe: false,
            json: false,
            presentation: crate::config::Presentation::Desktop,
            scope: Some("qualification".into()),
            connect: None,
            supervisor_id: Some("qualification".into()),
            run_id: Some("startup-fixture".into()),
            bound: None,
            auto_run: false,
        };
        let display = Arc::new(Mutex::new(DisplayState::default()));
        let receipt = fixtures.join("held-fixture.started");
        if held && receipt.exists() {
            std::fs::remove_file(&receipt).unwrap();
        }
        let mut owner = start_worker(options, display.clone()).unwrap();
        if held {
            wait(&display, |state| state.phase == "Waiting for supervisor");
            wait(&display, |_| receipt.exists());
            owner.cancel.cancel();
            owner.cancel.cancel();
        }
        let deadline = Instant::now() + Duration::from_secs(30);
        while !display.lock().unwrap().finished {
            assert!(
                Instant::now() < deadline,
                "startup cleanup did not complete"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
        let outcome = owner.thread.take().unwrap().join().unwrap();
        let state = display.lock().unwrap();
        assert_eq!(state.boundary, 0);
        assert!(!state.ready);
        assert!(state.finished);
        if held {
            assert!(outcome.unwrap().is_none());
            assert!(state.error.is_none());
            assert!(!state.cleanup_failed);
            let pid: i32 = std::fs::read_to_string(&receipt).unwrap().parse().unwrap();
            assert_eq!(unsafe { libc::kill(pid, 0) }, -1);
        } else {
            let error = outcome.unwrap_err();
            assert!(error.contains("TEST_SUPERVISOR_FAILURE"), "{error}");
            assert!(error.contains("exited before readiness"), "{error}");
            assert!(error.len() < 35 * 1024);
            assert!(
                state.cleanup_failed,
                "nonzero shutdown is retained as an unsuccessful cleanup outcome"
            );
        }
    }
}

#[test]
fn collapsed_cleanup_failure_requires_cleanup_and_reopen_without_impossible_retry() {
    for width in [480.0, 700.0] {
        let context = egui::Context::default();
        theme::apply(&context);
        let primary =
            "Supervisor startup failed: supervisor exited before readiness: exit status: 7";
        let state = DisplayState {
            error: Some(
                format!(
                    "{primary}\nTEST_SUPERVISOR_FAILURE\nCleanup: supervisor exited unsuccessfully"
                )
                .into(),
            ),
            cleanup_failed: true,
            finished: true,
            ..Default::default()
        };
        let mut output = context.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(width, 420.0),
                )),
                ..Default::default()
            },
            |ui| notice(ui, &state, None),
        );
        let text: Vec<_> = output
            .shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::Shape::Text(text) => Some(text.galley.text().to_owned()),
                _ => None,
            })
            .collect();
        assert!(text.iter().any(|line| line == primary));
        assert!(text.iter().any(|line| line.contains("Restart is disabled")));
        assert!(text.iter().any(|line| line.contains("before reopening")
            && line.contains("remote authority/session cleanup")));
        assert!(
            !text
                .iter()
                .any(|line| line.contains("then retry")
                    || line.contains("Open simulation or Restart"))
        );
        assert!(state.cleanup_failed);
        assert!(
            state
                .error
                .as_ref()
                .unwrap()
                .details
                .contains("TEST_SUPERVISOR_FAILURE")
        );
        output.textures_delta.clear();
    }
}

#[test]
fn collapsed_notice_keeps_complete_primary_cause_visible_before_long_path_details() {
    let reasons = [
        "MuJoCo 3.12.0 unavailable or incompatible: MuJoCo is incompatible: found MuJoCo 3.11.0; this simulator requires MuJoCo 3.12.0.",
        "Scene preparation failed: native model composition failed: native compile: Error: size 0 must be positive in geom; Element broken",
    ];
    for width in [480.0, 700.0] {
        for reason in reasons {
            let context = egui::Context::default();
            theme::apply(&context);
            let state = DisplayState {
                error: Some(
                    format!(
                        "{reason}\nSelected scene: /{}scene.xml",
                        "long path/".repeat(50)
                    )
                    .into(),
                ),
                ..Default::default()
            };
            let mut output = context.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(width, 420.0),
                    )),
                    ..Default::default()
                },
                |ui| {
                    notice(ui, &state, None);
                },
            );
            fn text(shapes: &[egui::epaint::ClippedShape]) -> Vec<String> {
                shapes
                    .iter()
                    .filter_map(|shape| match &shape.shape {
                        egui::Shape::Text(text) => Some(text.galley.text().to_owned()),
                        _ => None,
                    })
                    .collect()
            }
            let rendered = text(&output.shapes);
            assert!(
                rendered.iter().any(|text| text == reason),
                "complete cause was not rendered at {width}: {rendered:?}"
            );
            assert!(
                !rendered.iter().any(|text| text.contains("long path/")),
                "collapsed details must remain separate"
            );
            output.textures_delta.clear();
        }
    }
}

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
        theme::apply(&ctx);
        let (sender, _) = mpsc::channel(8);
        let mut desktop = Desktop {
            availability: None,
            control: Arc::new(Mutex::new(Control {
                cancel: crate::cancellation::Cancellation::default(),
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
            closing: false,
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
        theme::apply(&ctx);
        let (sender, _) = mpsc::channel(8);
        let mut desktop = Desktop {
            availability: None,
            control: Arc::new(Mutex::new(Control {
                cancel: crate::cancellation::Cancellation::default(),
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
                        parent: (id != 0).then_some(0),
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
                "A diagnostic remains accessible without consuming the viewport."
                    .repeat(4)
                    .into(),
            ),
            view_camera: None,
            gesture: None,
            next_gesture: 0,
            pointer_cut: None,
            closing: false,
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

#[test]
fn scene_hierarchy_selects_only_an_explicit_free_ancestor() {
    let bodies = vec![
        NativeBody {
            id: 0,
            name: "world".into(),
            mobility: BodyMobility::Fixed,
            parent: None,
        },
        NativeBody {
            id: 1,
            name: "root".into(),
            mobility: BodyMobility::FreeJoint,
            parent: Some(0),
        },
        NativeBody {
            id: 2,
            name: "wheel".into(),
            mobility: BodyMobility::Articulated,
            parent: Some(1),
        },
        NativeBody {
            id: 3,
            name: "floor".into(),
            mobility: BodyMobility::Fixed,
            parent: Some(0),
        },
    ];
    assert_eq!(free_ancestor(&bodies, 2), Some(1));
    assert_eq!(free_ancestor(&bodies, 1), None);
    assert_eq!(free_ancestor(&bodies, 3), None);
    assert_eq!(free_ancestor(&bodies, 9), None);
}

#[test]
fn collapsed_notice_projects_the_producing_phase_action() {
    for width in [480.0, 700.0] {
        for (primary, action) in [
            (
                "Build admission failed: invalid manifest",
                "select a complete runnable robot build directory, or rebuild that robot.",
            ),
            (
                "Scene preparation failed: size 0 must be positive",
                "repair the selected scene, model resources or component model, then reopen this scene and build.",
            ),
            (
                "Supervisor startup failed: child exit 7",
                "inspect the build's supervisor and participant errors, then retry.",
            ),
            (
                "Desktop graphics/window creation failed: no display",
                "run in a working graphical desktop session, or use `phoxal-simulator run --help` for explicit headless execution.",
            ),
            (
                "MuJoCo 3.12.0 unavailable or incompatible: The native library file or one of its dependencies was not found.",
                "repair or remove PHOXAL_MUJOCO_LIBRARY=/selected/missing/library and retry. This strict override takes precedence over managed setup.",
            ),
            (
                "MuJoCo 3.12.0 unavailable or incompatible: MuJoCo is incompatible: found MuJoCo 3.11.0; this simulator requires MuJoCo 3.12.0.",
                "run phoxal-simulator --runtime-root '/tmp/runtime directory' setup, then retry. Startup never downloads a runtime.",
            ),
        ] {
            let context = egui::Context::default();
            theme::apply(&context);
            let state = DisplayState {
                error: Some(Notice::new(
                    primary,
                    action,
                    format!(
                        "Selected path: /{}\nNative version detail: 3011000, required 3012000",
                        "long directory/".repeat(30)
                    ),
                )),
                ..Default::default()
            };
            let mut output = context.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(width, 420.0),
                    )),
                    ..Default::default()
                },
                |ui| notice(ui, &state, None),
            );
            let text: Vec<_> = output
                .shapes
                .iter()
                .filter_map(|shape| match &shape.shape {
                    egui::Shape::Text(text) => Some(text.galley.text().to_owned()),
                    _ => None,
                })
                .collect();
            for shape in &output.shapes {
                if let egui::Shape::Text(text) = &shape.shape {
                    assert!(
                        text.pos.y + text.galley.size().y <= 420.0,
                        "primary notice clipped at {width}"
                    );
                }
            }
            output.textures_delta.clear();
            assert!(
                text.iter().any(|line| line == &format!("Next: {action}")),
                "{width}: {text:?}"
            );
            assert!(text.iter().any(|line| line == primary));
            assert!(!text.iter().any(|line| line.contains("3011000")
                || line.contains("3012000")
                || line.contains("long directory/")));
        }
    }
}

#[test]
fn idle_availability_owner_is_nonblocking_isolated_and_cancellable() {
    let (entered, called) = std::sync::mpsc::channel();
    let (release, waiting) = std::sync::mpsc::channel();
    let mut check = AvailabilityCheck::spawn(move |_| {
        entered.send(()).unwrap();
        waiting.recv().unwrap();
        Err("MuJoCo unavailable\nNext: repair the selected library.".into())
    })
    .unwrap();
    called
        .recv_timeout(std::time::Duration::from_secs(5))
        .unwrap();
    assert!(check.result.lock().unwrap().is_none());
    let state = DisplayState {
        ready: true,
        cleanup_failed: true,
        generation: 42,
        ..Default::default()
    };
    let result = check.result.clone();
    release.send(()).unwrap();
    check.thread.take().unwrap().join().unwrap();
    assert!(result.lock().unwrap().as_ref().unwrap().is_err());
    assert!(state.ready && state.cleanup_failed);
    assert_eq!(state.generation, 42);

    let (entered, called) = std::sync::mpsc::channel();
    let check = AvailabilityCheck::spawn(move |cancel| {
        entered.send(()).unwrap();
        tokio::runtime::Builder::new_current_thread()
            .build()
            .unwrap()
            .block_on(cancel.wait());
        cancel.check().map_err(Notice::from)
    })
    .unwrap();
    called
        .recv_timeout(std::time::Duration::from_secs(5))
        .unwrap();
    let result = check.result.clone();
    check.finish().unwrap();
    assert!(
        result.lock().unwrap().is_none(),
        "cancelled availability never publishes a failure"
    );
}

#[test]
fn late_idle_availability_cannot_replace_execution_notice_or_cleanup_fence() {
    let context = egui::Context::default();
    theme::apply(&context);
    let (commands, _) = mpsc::channel(1);
    let display = Arc::new(Mutex::new(DisplayState {
        finished: true,
        cleanup_failed: true,
        generation: 42,
        error: Some("Supervisor startup failed: exit 7\nNext: inspect the supervisor.".into()),
        ..Default::default()
    }));
    let mut desktop = Desktop {
        availability: Some(Arc::new(Mutex::new(Some(Err(
            "LATE IDLE FAILURE\nNext: repair native library.".into(),
        ))))),
        control: Arc::new(Mutex::new(Control {
            commands,
            cancel: crate::cancellation::Cancellation::default(),
            thread: None,
        })),
        display: display.clone(),
        options: Some(Options {
            simulation_run: None,
            probe: false,
            scene: "scene.xml".into(),
            bundle: "build".into(),
            json: false,
            presentation: crate::config::Presentation::Desktop,
            scope: None,
            connect: None,
            supervisor_id: None,
            run_id: None,
            bound: None,
            auto_run: true,
        }),
        build_path: "build".into(),
        scene_path: "scene.xml".into(),
        restart: 0,
        texture: None,
        message: None,
        view_camera: None,
        gesture: None,
        next_gesture: 0,
        pointer_cut: None,
        closing: false,
        last_viewport: None,
    };
    let mut output = context.run_ui(
        egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(700.0, 552.0),
            )),
            ..Default::default()
        },
        |ui| desktop.draw(ui),
    );
    let text: Vec<_> = output
        .shapes
        .iter()
        .filter_map(|shape| match &shape.shape {
            egui::Shape::Text(text) => Some(text.galley.text().to_owned()),
            _ => None,
        })
        .collect();
    output.textures_delta.clear();
    assert!(
        text.iter()
            .any(|line| line.contains("Supervisor startup failed: exit 7")),
        "{text:?}"
    );
    assert!(
        !text.iter().any(
            |line| line.contains("LATE IDLE FAILURE") || line.contains("repair native library")
        )
    );
    let state = display.lock().unwrap();
    assert!(state.finished && state.cleanup_failed);
    assert_eq!(state.generation, 42);
    assert!(state.error.as_ref().unwrap().details.contains("exit 7"));
}

#[test]
fn secondary_next_text_cannot_replace_primary_recovery() {
    let native_action = "repair or remove PHOXAL_MUJOCO_LIBRARY=/selected/missing\nNext: SECONDARY_PATH_TEXT and retry. This strict override takes precedence over managed setup.";
    let native = Notice::new(
        "MuJoCo unavailable: file not found",
        native_action,
        "Candidates:\nlibrary discovery at /selected/missing\nNext: SECONDARY_PATH_TEXT: dlopen failed",
    );
    let graphics = graphics_notice(
        "display unavailable".into(),
        Some("Build failed\nNext: SECONDARY_BUILD_ACTION".into()),
    );
    assert_eq!(
        desktop_outcome::<()>(
            Err("display unavailable".into()),
            Err("Build failed\nNext: SECONDARY_BUILD_ACTION".into())
        )
        .unwrap_err(),
        graphics.to_string()
    );
    let child = Notice::new(
        "Supervisor startup failed: exit 7",
        "inspect the supervisor and participants.",
        "Child stderr:\nNext: SECONDARY_CHILD_ACTION\nAdditional cleanup details:\nNext: SECONDARY_CLEANUP_ACTION",
    );
    for width in [480.0, 700.0] {
        for failure in [&native, &graphics, &child] {
            let context = egui::Context::default();
            theme::apply(&context);
            let state = DisplayState {
                error: Some(failure.clone()),
                ..Default::default()
            };
            let mut output = context.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(width, 420.0),
                    )),
                    ..Default::default()
                },
                |ui| notice(ui, &state, None),
            );
            let text: Vec<_> = output
                .shapes
                .iter()
                .filter_map(|shape| match &shape.shape {
                    egui::Shape::Text(t) => Some(t.galley.text().to_owned()),
                    _ => None,
                })
                .collect();
            output.textures_delta.clear();
            assert!(
                text.iter()
                    .any(|line| line == &format!("Next: {}", failure.action)),
                "{width}: {text:?}"
            );
            assert!(
                !text
                    .iter()
                    .any(|line| line == "Next: SECONDARY_BUILD_ACTION"
                        || line == "Next: SECONDARY_PATH_TEXT: dlopen failed"
                        || line == "Next: SECONDARY_CHILD_ACTION")
            );
        }
    }
}

#[test]
fn idle_runtime_failure_primary_and_action_are_visible_in_the_full_narrow_desktop() {
    let primary = "MuJoCo 3.12.0 unavailable or incompatible: The native library file or one of its dependencies was not found.";
    let action = "repair or remove PHOXAL_MUJOCO_LIBRARY=/selected/native runtime directories/another long containing directory/mujoco.framework/Versions/A/missing-library.dylib and retry. This strict override takes precedence over managed setup.";
    for height in [452.0, 420.0] {
        let ctx = egui::Context::default();
        theme::apply(&ctx);
        let (commands, _) = mpsc::channel(1);
        let mut desktop = Desktop {
            availability: Some(Arc::new(Mutex::new(Some(Err(Notice::new(
                primary,
                action,
                "Selected library: /selected/missing/library",
            )))))),
            control: Arc::new(Mutex::new(Control {
                commands,
                cancel: crate::cancellation::Cancellation::default(),
                thread: None,
            })),
            display: Arc::new(Mutex::new(DisplayState {
                finished: true,
                ..Default::default()
            })),
            options: None,
            build_path: String::new(),
            scene_path: String::new(),
            restart: 0,
            texture: None,
            message: None,
            view_camera: None,
            gesture: None,
            next_gesture: 0,
            pointer_cut: None,
            closing: false,
            last_viewport: None,
        };
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(480.0, height));
        let mut output = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        output.textures_delta.clear();
        for expected in [primary.to_owned(), format!("Next: {action}")] {
            let rect = output.shapes.iter().find_map(|shape| {
                if let egui::Shape::Text(text) = &shape.shape
                    && text.galley.text() == expected
                {
                    Some((
                        egui::Rect::from_min_size(text.pos, text.galley.size()),
                        shape.clip_rect,
                    ))
                } else {
                    None
                }
            });
            println!("Full idle desktop 480x{height}, {expected}: painter geometry {rect:?}");
            let (rect, clip) =
                rect.unwrap_or_else(|| panic!("primary text not painted: {expected}"));
            assert!(
                screen.contains_rect(rect) && clip.contains_rect(rect),
                "primary/action is clipped at 480x{height}: {rect:?}, clip {clip:?}"
            );
        }
        let point = output
            .shapes
            .iter()
            .find_map(|shape| {
                if let egui::Shape::Text(text) = &shape.shape
                    && text.galley.text() == "Diagnostics"
                {
                    Some(text.pos + egui::vec2(3.0, 3.0))
                } else {
                    None
                }
            })
            .unwrap_or_else(|| panic!("Diagnostics header unavailable at 480x{height}"));
        let events = vec![
            egui::Event::PointerMoved(point),
            egui::Event::PointerButton {
                pos: point,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: Default::default(),
            },
            egui::Event::PointerButton {
                pos: point,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: Default::default(),
            },
        ];
        let mut clicked = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                events,
                time: Some(1.0),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        clicked.textures_delta.clear();
        let mut expanded = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                time: Some(2.0),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        expanded.textures_delta.clear();
        assert!(expanded.shapes.iter().any(|shape| {
            matches!(&shape.shape, egui::Shape::Text(text) if text.galley.text().starts_with("Boundary "))
        }), "Diagnostics click did not expand its real body");
        for expected in [primary.to_owned(), format!("Next: {action}")] {
            let (rect, clip) = expanded
                .shapes
                .iter()
                .find_map(|shape| {
                    if let egui::Shape::Text(text) = &shape.shape
                        && text.galley.text() == expected
                    {
                        Some((
                            egui::Rect::from_min_size(text.pos, text.galley.size()),
                            shape.clip_rect,
                        ))
                    } else {
                        None
                    }
                })
                .unwrap_or_else(|| panic!("primary missing after Diagnostics expansion"));
            println!(
                "Expanded idle desktop 480x{height}: primary/action rect={rect:?}, clip={clip:?}"
            );
            assert!(screen.contains_rect(rect) && clip.contains_rect(rect));
        }
        output.textures_delta.clear();
    }
}

#[test]
fn focused_path_field_paints_white_frame_without_changing_teal_selection() {
    for adapted in [false, true] {
        let ctx = egui::Context::default();
        theme::apply(&ctx);
        let selection = ctx.style_of(egui::Theme::Dark).visuals.selection;
        let mut value = "Selected path".to_owned();
        let mut field = None;
        let mut first = ctx.run_ui(Default::default(), |ui| {
            let response = if adapted {
                path_input(ui, &mut value)
            } else {
                ui.add(egui::TextEdit::singleline(&mut value))
            };
            field = Some((response.id, response.rect));
        });
        first.textures_delta.clear();
        ctx.memory_mut(|memory| memory.request_focus(field.unwrap().0));
        let mut output = ctx.run_ui(Default::default(), |ui| {
            let response = if adapted {
                path_input(ui, &mut value)
            } else {
                ui.add(egui::TextEdit::singleline(&mut value))
            };
            assert!(response.has_focus());
            field = Some((response.id, response.rect));
        });
        output.textures_delta.clear();
        let rect = field.unwrap().1;
        let strokes: Vec<_> = output
            .shapes
            .iter()
            .filter_map(|shape| {
                if let egui::Shape::Rect(paint) = &shape.shape
                    && paint.rect.intersects(rect)
                    && paint.stroke.width > 0.0
                {
                    Some(paint.stroke.color)
                } else {
                    None
                }
            })
            .collect();
        println!("TextEdit adapted={adapted}, actual frame strokes={strokes:?}");
        assert_eq!(ctx.style_of(egui::Theme::Dark).visuals.selection, selection);
        if adapted {
            assert!(strokes.contains(&egui::Color32::WHITE));
        } else {
            assert!(strokes.contains(&selection.stroke.color));
            assert!(!strokes.contains(&egui::Color32::WHITE));
        }
    }
}
