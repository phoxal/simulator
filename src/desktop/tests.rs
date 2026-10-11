use super::*;
#[cfg(any(native_qualification, external_source_qualification))]
use std::time::{Duration, Instant};

type ChromeDraw = dyn FnMut(Vec<egui::Event>) -> Vec<String>;

fn chrome_harness(
    width: f32,
    height: f32,
) -> (
    egui::Context,
    std::rc::Rc<std::cell::RefCell<Desktop>>,
    std::rc::Rc<std::cell::RefCell<ChromeDraw>>,
) {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let display = Arc::new(Mutex::new(RunState::default()));
    {
        let mut state = display.lock().unwrap();
        state.ready = true;
        state.finished = true;
        state
            .document
            .attachments
            .push(("robot_mount".into(), [0.; 3], [1., 0., 0., 0.]));
    }
    let world = Arc::new(Mutex::new(Handle::fixture(display.clone())));
    let desktop = std::rc::Rc::new(std::cell::RefCell::new(Desktop::new(world, display)));
    desktop.borrow_mut().texture = Some(ctx.load_texture(
        "chrome",
        egui::ColorImage::filled([64, 48], egui::Color32::BLACK),
        egui::TextureOptions::LINEAR,
    ));
    let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, height));
    let render = ctx.clone();
    let owned = std::rc::Rc::clone(&desktop);
    let draw = move |events| {
        render.data_mut(|d| {
            d.remove::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                "control-probes",
            ))
        });
        let mut output = render.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                events,
                ..Default::default()
            },
            |ui| owned.borrow_mut().draw(ui),
        );
        render.data_mut(|d| {
            d.insert_temp(egui::Id::new("clipped-shape-probes"), output.shapes.clone())
        });
        output.textures_delta.clear();
        // Clipping-aware probes: text is admitted only when its visual
        // bounds fit inside the shape's actual clip rectangle (0.5pt
        // tolerance), so offscreen or clipped labels cannot satisfy
        // visibility assertions.
        let pairs: Vec<(String, egui::Rect)> = output
            .shapes
            .iter()
            .filter_map(|shape| {
                if let egui::Shape::Text(text) = &shape.shape {
                    let bounds = shape.shape.visual_bounding_rect();
                    let clip = shape.clip_rect.expand(0.5);
                    clip.contains_rect(bounds)
                        .then_some((text.galley.text().to_owned(), bounds))
                } else {
                    None
                }
            })
            .collect();
        render.data_mut(|d| d.insert_temp(egui::Id::new("text-probes"), pairs.clone()));
        #[cfg(test)]
        render.data_mut(|d| {
            let plain: Vec<egui::Shape> = output.shapes.iter().map(|s| s.shape.clone()).collect();
            d.insert_temp(egui::Id::new("shape-probes"), plain)
        });
        pairs.into_iter().map(|(text, _)| text).collect()
    };
    (
        ctx,
        desktop,
        std::rc::Rc::new(std::cell::RefCell::new(draw)),
    )
}

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
#[cfg(native_qualification)]
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
        let display = Arc::new(Mutex::new(RunState::default()));
        let receipt = fixtures.join("held-fixture.started");
        if held && receipt.exists() {
            std::fs::remove_file(&receipt).unwrap();
        }
        let mut owner = Handle::qualification(options, display.clone()).unwrap();
        if held {
            wait(&display, |state| {
                state.phase == crate::app::RunStatus::WaitingForSupervisor
            });
            wait(&display, |_| receipt.exists());
            owner.send(Command::Stop).unwrap();
            owner.send(Command::Stop).unwrap();
        }
        let deadline = Instant::now() + Duration::from_secs(30);
        while !display.lock().unwrap().finished {
            assert!(
                Instant::now() < deadline,
                "startup cleanup did not complete"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
        let outcome = owner.shutdown();
        let state = display.lock().unwrap();
        assert_eq!(state.boundary, 0);
        assert!(!state.ready);
        assert!(state.finished);
        if held {
            assert!(matches!(outcome.unwrap(), RunOutcome::Cancelled));
            assert!(state.error.is_none());
            assert!(!state.cleanup_failed);
            let pid: i32 = std::fs::read_to_string(&receipt).unwrap().parse().unwrap();
            assert_eq!(unsafe { libc::kill(pid, 0) }, -1);
        } else {
            let error = outcome.err().unwrap();
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
        let state = RunState {
 snapshot: crate::app::RunSnapshot {
            error: Some(
                format!(
                    "{primary}\nTEST_SUPERVISOR_FAILURE\nCleanup: supervisor exited unsuccessfully"
                )
                .into(),
            ),
            cleanup_failed: true,
            finished: true,
 ..Default::default()
 },
 requests: crate::app::RunRequests {
 ..Default::default()
 },
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
            let state = RunState {
                snapshot: crate::app::RunSnapshot {
                    error: Some(
                        format!(
                            "{reason}\nSelected scene: /{}scene.xml",
                            "long path/".repeat(50)
                        )
                        .into(),
                    ),
                    ..Default::default()
                },
                requests: crate::app::RunRequests {
                    ..Default::default()
                },
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
#[cfg(native_qualification)]
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
    let display = Arc::new(Mutex::new(RunState::default()));
    let mut owner = Handle::qualification(options.clone(), display.clone()).unwrap();
    wait(&display, |state| state.ready);
    assert_eq!(display.lock().unwrap().pacing, Pacing::Realtime);
    assert_eq!(display.lock().unwrap().wall_seconds, 0.0);
    owner.send(Command::Step).unwrap();
    wait(&display, |state| state.boundary == 1);
    assert!(!display.lock().unwrap().running);
    let stepped_wall = display.lock().unwrap().wall_seconds;
    std::thread::sleep(Duration::from_millis(1100));
    assert_eq!(display.lock().unwrap().boundary, 1);
    assert_eq!(display.lock().unwrap().wall_seconds, stepped_wall);
    owner.send(Command::Run).unwrap();
    wait(&display, |state| state.boundary >= 101);
    let recent = display.lock().unwrap().speed;
    owner.send(Command::Pause).unwrap();
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
    owner.send(Command::SetPacing(Pacing::Fast)).unwrap();
    wait(&display, |state| state.pacing == Pacing::Fast);
    let fast_start = {
        let state = display.lock().unwrap();
        (state.boundary, state.time_seconds, state.wall_seconds)
    };
    owner.send(Command::Run).unwrap();
    wait(&display, |state| state.boundary >= fast_start.0 + 100);
    owner.send(Command::Pause).unwrap();
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
    owner.send(Command::SetPacing(Pacing::Realtime)).unwrap();
    wait(&display, |state| state.pacing == Pacing::Realtime);
    let before = display.lock().unwrap().boundary;
    owner.send(Command::Step).unwrap();
    wait(&display, |state| state.boundary == before + 1);
    owner.send(Command::Reset).unwrap();
    wait(&display, |state| state.boundary == 0);
    {
        let state = display.lock().unwrap();
        assert_eq!(state.time_seconds, 0.0);
        assert_eq!(state.wall_seconds, 0.0);
        assert_eq!(state.speed, None);
        assert!(!state.running);
    }
    owner.send(Command::Run).unwrap();
    wait(&display, |state| state.boundary >= 5);
    owner.send(Command::Pause).unwrap();
    wait(&display, |state| !state.running);
    assert!(display.lock().unwrap().wall_seconds > 0.0);
    owner.send(Command::Stop).unwrap();
    wait(&display, |state| state.finished);
    assert!(owner.shutdown().is_ok());
    assert!(!display.lock().unwrap().ready);
    assert_eq!(display.lock().unwrap().speed, None);
    let mut options = options;
    options.run_id = Some("pacing-qualification-restart".into());
    options.auto_run = false;
    *display.lock().unwrap() = RunState::default();
    owner = Handle::qualification(options, display.clone()).unwrap();
    wait(&display, |state| state.ready);
    {
        let state = display.lock().unwrap();
        assert_eq!(state.pacing, Pacing::Realtime);
        assert_eq!(state.wall_seconds, 0.0);
        assert_eq!(state.boundary, 0);
    }
    owner.send(Command::Run).unwrap();
    wait(&display, |state| state.boundary >= 1);
    let (disconnected, _) = mpsc::channel(1);
    owner.control.lock().unwrap().commands = disconnected;
    wait(&display, |state| state.finished);
    assert!(owner.shutdown().is_ok());
    println!(
        "CONTROL pause-stable, step, run, both pacing acknowledgements, reset, stop, restart and channel-close joined cleanup passed"
    );
}

#[cfg(any(native_qualification, external_source_qualification))]
pub(super) fn wait(display: &Arc<Mutex<RunState>>, predicate: impl Fn(&RunState) -> bool) {
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

#[test]
#[cfg(native_qualification)]
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
    let display = Arc::new(Mutex::new(RunState::default()));
    let mut owner = Handle::qualification(options.clone(), display.clone()).unwrap();
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        wait(&display, |state| state.ready && state.frame.is_some());
        let (before, body, original) = {
            let mut state = display.lock().unwrap();
            let frame = state.frame.take().unwrap();
            save_native_frame("before", &frame.image);
            let original = frame.view.clone();
            state.requests.presented = Some(frame.view);
            let body = state
                .bodies
                .iter()
                .find(|body| body.mobility.label() == "Free joint")
                .unwrap()
                .id;
            queue_scene(&mut state, scene::SceneAction::SelectBody(body));
            (
                state.requests.presented.as_ref().unwrap().snapshot.clone(),
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
            state.requests.presented = Some(frame.view);
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
            state.requests.presented = Some(frame.view);
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
        owner.send(Command::Step).unwrap();
        wait(&display, |state| state.boundary == 1);
        owner.send(Command::Reset).unwrap();
        wait(&display, |state| {
            state.boundary == 0 && state.generation > 0
        });
        {
            let mut state = display.lock().unwrap();
            assert!(state.selection.is_none());
            state.requests.presented = Some(original.clone());
            state.requests.pending_scene = Some(scene::SceneRequest {
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
        .requests
        .presented
        .as_ref()
        .map(|view| view.identity.epoch.execution.clone());
    let _ = owner.send(Command::Stop);
    assert!(owner.shutdown().is_ok());
    assert!(display.lock().unwrap().requests.presented.is_none());
    assert!(display.lock().unwrap().selection.is_none());
    if let Err(payload) = outcome {
        std::panic::resume_unwind(payload);
    }
    let original_epoch = display.lock().unwrap().generation;
    options.run_id = Some("scene-selection-proof-restart".into());
    *display.lock().unwrap() = RunState::default();
    let mut restarted = Handle::qualification(options, display.clone()).unwrap();
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
    let _ = restarted.send(Command::Stop);
    assert!(restarted.shutdown().is_ok());
    if let Err(payload) = result {
        std::panic::resume_unwind(payload);
    }
}
#[cfg(native_qualification)]
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
#[cfg(native_qualification)]
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
    let display = Arc::new(Mutex::new(RunState::default()));
    let mut owner = Handle::qualification(options.clone(), display.clone()).unwrap();
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        wait(&display, |s| s.ready && s.frame.is_some());
        let original = {
            let mut s = display.lock().unwrap();
            let frame = s.frame.take().unwrap();
            save_native_frame("drag-original", &frame.image);
            let original = frame.view.snapshot.clone();
            s.requests.presented = Some(frame.view);
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
            s.requests.presented = Some(s.frame.take().unwrap().view);
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
            s.requests.presented = Some(s.frame.take().unwrap().view);
            let view = s.requests.presented.as_ref().unwrap().clone();
            s.requests.drag_mailbox.begin = Some(scene::DragBegin {
                id: 1,
                frame: view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
            s.requests.drag_mailbox.update = Some(scene::DragUpdate {
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
            s.requests.presented = Some(f.view);
            s.requests.drag_mailbox.end(1);
            edited
        };
        wait(&display, |s| !s.dragging);
        owner.send(Command::Step).unwrap();
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
        owner.send(Command::Run).unwrap();
        wait(&display, |s| s.boundary >= 5);
        owner.send(Command::Pause).unwrap();
        wait(&display, |s| !s.running);
        let boundary = display.lock().unwrap().boundary;
        wait(&display, |s| {
            s.frame
                .as_ref()
                .is_some_and(|f| f.view.snapshot.boundary() == boundary)
        });
        {
            let mut s = display.lock().unwrap();
            s.requests.presented = Some(s.frame.take().unwrap().view);
            let view = s.requests.presented.as_ref().unwrap().clone();
            s.requests.drag_mailbox.begin = Some(scene::DragBegin {
                id: 2,
                frame: view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
        }
        wait(&display, |s| s.dragging);
        // Run cancels a paused pose gesture; a fresh gesture is physically applied.
        owner.send(Command::Run).unwrap();
        wait(&display, |s| s.running && !s.dragging && s.frame.is_some());
        {
            let mut s = display.lock().unwrap();
            s.requests.presented = Some(s.frame.take().unwrap().view);
            let view = s.requests.presented.as_ref().unwrap().clone();
            s.requests.drag_mailbox.begin = Some(scene::DragBegin {
                id: 3,
                frame: view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
            s.requests.drag_mailbox.update = Some(scene::DragUpdate {
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
            assert!(display.lock().unwrap().dragging);
            let mut s = display.lock().unwrap();
            s.requests.drag_mailbox.update = Some(scene::DragUpdate {
                id: 3,
                epoch: s
                    .requests
                    .presented
                    .as_ref()
                    .unwrap()
                    .identity
                    .epoch
                    .clone(),
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
            s.requests.drag_mailbox.update = Some(scene::DragUpdate {
                id: 3,
                epoch: s
                    .requests
                    .presented
                    .as_ref()
                    .unwrap()
                    .identity
                    .epoch
                    .clone(),
                delta: [0.08, 0.0],
                received: Instant::now(),
            });
        }
        wait(&display, |s| s.requests.drag_mailbox.update.is_none());
        assert!(!display.lock().unwrap().dragging);
        owner.send(Command::Pause).unwrap();
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
        owner.send(Command::Reset).unwrap();
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
            s.requests.presented = Some(original_view.clone());
            s.requests.drag_mailbox.begin = Some(scene::DragBegin {
                id: 4,
                frame: original_view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
        }
        wait(&display, |s| s.interaction_error.is_some());
        assert!(!display.lock().unwrap().dragging);
    }));
    let stale_view = display.lock().unwrap().requests.presented.clone();
    let _ = owner.send(Command::Stop);
    assert!(owner.shutdown().is_ok());
    assert!(!display.lock().unwrap().dragging);
    if let Err(payload) = outcome {
        std::panic::resume_unwind(payload);
    }
    *display.lock().unwrap() = RunState::default();
    let mut restarted = Handle::qualification(options, display.clone()).unwrap();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        wait(&display, |s| s.ready && s.frame.is_some());
        if let Some(view) = stale_view {
            let mut s = display.lock().unwrap();
            s.requests.presented = Some(view.clone());
            s.requests.drag_mailbox.begin = Some(scene::DragBegin {
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
            s.requests.presented = Some(s.frame.take().unwrap().view);
            queue_scene(&mut s, scene::SceneAction::SelectBody(root));
        }
        wait(&display, |s| s.selection.is_some() && s.frame.is_some());
        {
            let mut s = display.lock().unwrap();
            s.requests.presented = Some(s.frame.take().unwrap().view);
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
            s.requests.presented = Some(s.frame.take().unwrap().view);
            let view = s.requests.presented.as_ref().unwrap().clone();
            s.requests.drag_mailbox.begin = Some(scene::DragBegin {
                id: 6,
                frame: view.identity.clone(),
                xy: [0.5; 2],
                received: Instant::now(),
            });
            s.requests.drag_mailbox.update = Some(scene::DragUpdate {
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
        restarted.control.lock().unwrap().commands = disconnected;
        wait(&display, |s| s.finished);
        assert!(!display.lock().unwrap().dragging);
    }));
    let _ = restarted.send(Command::Stop);
    assert!(restarted.shutdown().is_ok());
    if let Err(payload) = result {
        std::panic::resume_unwind(payload);
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
                "reinstall the complete Phoxal Simulator application.",
            ),
            (
                "MuJoCo 3.12.0 unavailable or incompatible: MuJoCo is incompatible: found MuJoCo 3.11.0; this simulator requires MuJoCo 3.12.0.",
                "reinstall the complete Phoxal Simulator application.",
            ),
        ] {
            let context = egui::Context::default();
            theme::apply(&context);
            let state = RunState {
                snapshot: crate::app::RunSnapshot {
                    error: Some(Notice::new(
                        primary,
                        action,
                        format!(
                            "Selected path: /{}\nNative version detail: 3011000, required 3012000",
                            "long directory/".repeat(30)
                        ),
                    )),
                    ..Default::default()
                },
                requests: crate::app::RunRequests {
                    ..Default::default()
                },
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
fn secondary_next_text_cannot_replace_primary_recovery() {
    let native_action =
        "reinstall the complete application\nNext: SECONDARY_PATH_TEXT is secondary detail.";
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
            let state = RunState {
                snapshot: crate::app::RunSnapshot {
                    error: Some(failure.clone()),
                    ..Default::default()
                },
                requests: crate::app::RunRequests {
                    ..Default::default()
                },
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
fn scene_first_canvas_is_dominant_and_notices_do_not_move_it() {
    for [width, height] in [[320., 420.], [480., 420.], [736., 560.], [1200., 820.]] {
        let ctx = egui::Context::default();
        theme::apply(&ctx);
        let display = Arc::new(Mutex::new(RunState {
            snapshot: crate::app::RunSnapshot {
                ready: true,
                finished: true,
                ..Default::default()
            },
            requests: Default::default(),
        }));
        let world = Arc::new(Mutex::new(Handle::fixture(display.clone())));
        let mut desktop = Desktop::new(world, display);
        desktop.texture = Some(ctx.load_texture(
            "canvas",
            egui::ColorImage::filled([64, 48], egui::Color32::BLACK),
            egui::TextureOptions::LINEAR,
        ));
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, height));
        let mut baseline = None;
        for notice in [false, true, false] {
            desktop.message=notice.then(||Notice::new("A long diagnostic wraps above the controls without changing the native canvas.","repair the selected input", "full details"));
            for _ in 0..3 {
                let mut output = ctx.run_ui(
                    egui::RawInput {
                        screen_rect: Some(screen),
                        ..Default::default()
                    },
                    |ui| desktop.draw(ui),
                );
                output.textures_delta.clear();
            }
            let canvas = desktop.last_viewport.unwrap();
            assert!(screen.contains_rect(canvas));
            assert!(canvas.height() > height * 0.6);
            assert!(canvas.width() > width * 0.9);
            if let Some(prior) = baseline {
                assert_eq!(canvas, prior);
            } else {
                baseline = Some(canvas);
            }
            assert!(!desktop.inspector_open && !desktop.console_open);
        }
    }
}

#[test]
fn egui_close_request_exits_without_clearing_cleanup_uncertainty_or_restart_fence() {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let display = Arc::new(Mutex::new(RunState::default()));
    {
        let mut state = display.lock().unwrap();
        state.finished = true;
        state.cleanup_failed = true;
        state.error = Some(Notice::new(
            "Primary execution failure",
            "close and resolve remote cleanup",
            "remote release unconfirmed",
        ));
    }
    let world = Arc::new(Mutex::new(Handle::fixture(display.clone())));
    let mut desktop = Desktop::new(world.clone(), display.clone());
    let mut input = egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::vec2(736., 560.),
        )),
        ..Default::default()
    };
    input
        .viewports
        .get_mut(&egui::ViewportId::ROOT)
        .unwrap()
        .events
        .push(egui::ViewportEvent::Close);
    let mut output = ctx.run_ui(input, |ui| desktop.draw(ui));
    output.textures_delta.clear();
    assert!(
        desktop.allow_close,
        "cleanup uncertainty made the native close request impossible"
    );
    assert!(
        output.viewport_output[&egui::ViewportId::ROOT]
            .commands
            .contains(&egui::ViewportCommand::Close)
    );
    let state = display.lock().unwrap();
    assert!(state.cleanup_failed);
    assert_eq!(
        state.error.as_ref().unwrap().primary,
        "Primary execution failure"
    );
    drop(state);
    assert!(world.lock().unwrap().send(Command::Run).is_err());
    assert!(world.lock().unwrap().edit(Edit::New).is_err());
    let mut departure = Departure::default();
    departure.begin(PendingAction::New);
    assert!(departure.take_ready(true, true).is_none());
}

#[test]
fn actual_control_geometry_and_modal_tab_escape_focus_restore_at_narrow_widths() {
    for (width, large) in [320., 480., 736., 1024.]
        .into_iter()
        .flat_map(|w| [(w, false), (w, true)])
    {
        let ctx = egui::Context::default();
        theme::apply(&ctx);
        let display = Arc::new(Mutex::new(RunState::default()));
        {
            let mut s = display.lock().unwrap();
            s.ready = true;
            s.finished = true;
        }
        let world = Arc::new(Mutex::new(Handle::fixture(display.clone())));
        let mut desktop = Desktop::new(world, display);
        desktop.larger_targets = large;
        desktop.texture = Some(ctx.load_texture(
            "geometry",
            egui::ColorImage::filled([64, 48], egui::Color32::BLACK),
            egui::TextureOptions::LINEAR,
        ));
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, 560.));
        for _ in 0..4 {
            ctx.data_mut(|d| {
                d.remove::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                    "control-probes",
                ))
            });
            let mut output = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(screen),
                    ..Default::default()
                },
                |ui| desktop.draw(ui),
            );
            output.textures_delta.clear();
        }
        let buttons = ctx
            .data(|d| {
                d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                    "control-probes",
                ))
            })
            .unwrap();
        for (name, rect, ..) in &buttons {
            assert!(
                screen.contains_rect(*rect),
                "{width}: {name} outside screen {rect:?}"
            );
            assert_eq!(
                rect.size(),
                egui::vec2(if large { 40. } else { 30. }, if large { 40. } else { 30. }),
                "{width}: {name}"
            );
        }
        for rect in &desktop.captured[..2] {
            assert!(
                (rect.height() - if large { 52. } else { 42. }).abs() < 0.1,
                "{width} large={large}: toolbar {rect:?} viewport={:?}",
                desktop.last_viewport
            );
            assert!(screen.contains_rect(*rect));
        }
        let canvas = desktop.last_viewport.unwrap();
        let transport = desktop.captured[0];
        let camera = desktop.captured[1];
        assert!(
            (canvas.right() - transport.right() - 16.).abs() < 0.1,
            "{width}: transport edge {:?} canvas {:?}",
            transport,
            canvas
        );
        assert!((camera.left() - canvas.left() - 16.).abs() < 0.1);
        assert!(
            (canvas.bottom() - transport.bottom() - 16.).abs() < 0.6,
            "{width}: transport bottom {:?} canvas {:?} (half-pixel snapping allowed)",
            transport,
            canvas
        );
        if camera.top() < transport.top() {
            assert!((transport.top() - camera.bottom() - 8.).abs() < 0.1);
        }
        assert!(!desktop.captured[0].intersects(desktop.captured[1]));
        let help = buttons.iter().find(|b| b.0 == "Help").unwrap().2;
        ctx.memory_mut(|m| m.request_focus(help));
        desktop.help_open = true;
        let mut output = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        output.textures_delta.clear();
        let initial = ctx.memory(|m| m.focused()).unwrap();
        assert_ne!(initial, help);
        for shift in [false, false, true, true] {
            let mut output = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(screen),
                    events: vec![egui::Event::Key {
                        key: egui::Key::Tab,
                        physical_key: None,
                        pressed: true,
                        repeat: false,
                        modifiers: egui::Modifiers {
                            shift,
                            ..Default::default()
                        },
                    }],
                    ..Default::default()
                },
                |ui| desktop.draw(ui),
            );
            output.textures_delta.clear();
            assert_ne!(
                ctx.memory(|m| m.focused()),
                Some(help),
                "Tab escaped actual modal"
            );
        }
        let mut output = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                events: vec![egui::Event::Key {
                    key: egui::Key::Escape,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: Default::default(),
                }],
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        output.textures_delta.clear();
        assert!(!desktop.help_open);
        // One retry frame: the modal owner disabled the background while
        // closing, so the restore lands once the invoker is enabled again.
        let mut output = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        output.textures_delta.clear();
        assert_eq!(ctx.memory(|m| m.focused()), Some(help));
    }
}

#[test]
#[cfg(native_qualification)]
fn reachable_draw_continues_orbit_and_shift_pan_with_delayed_native_presentation() {
    for shift in [false, true] {
        let ctx = egui::Context::default();
        theme::apply(&ctx);
        let model = crate::mujoco::Model::from_closed(
            crate::document::Document::default_scene()
                .unwrap()
                .definition
                .scene
                .close()
                .unwrap(),
        )
        .unwrap();
        let scene = crate::mujoco::Scene::new(model.clone()).unwrap();
        let before = scene.snapshot().unwrap();
        let mut native_view =
            crate::execution::viewport::Viewport::new(&model, "delayed-presentation", 0).unwrap();
        let frame = native_view.render(&before).unwrap();
        let presented = frame.view.clone();
        let display = Arc::new(Mutex::new(RunState::default()));
        {
            let mut state = display.lock().unwrap();
            state.ready = true;
            state.finished = true;
            state.frame = Some(frame);
        }
        let world = Arc::new(Mutex::new(Handle::fixture(display.clone())));
        let mut desktop = Desktop::new(world, display.clone());
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(736., 560.));
        let mut time = 0.;
        let mut draw = |desktop: &mut Desktop, events| {
            time += 0.1;
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(screen),
                    time: Some(time),
                    events,
                    ..Default::default()
                },
                |ui| {
                    ui.ctx().input_mut(|i| i.modifiers.shift = shift);
                    desktop.draw(ui)
                },
            );
            out.textures_delta.clear();
        };
        for _ in 0..3 {
            draw(&mut desktop, vec![]);
        }
        let start = desktop.last_viewport.unwrap().center();
        let modifiers = egui::Modifiers {
            shift,
            ..Default::default()
        };
        draw(
            &mut desktop,
            vec![
                egui::Event::PointerMoved(start),
                egui::Event::PointerButton {
                    pos: start,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers,
                },
            ],
        );
        let initial = desktop.view_camera.unwrap();
        for delta in [
            egui::vec2(30., 15.),
            egui::vec2(60., 30.),
            egui::vec2(90., 45.),
        ] {
            let prior = desktop.view_camera.unwrap();
            draw(&mut desktop, vec![egui::Event::PointerMoved(start + delta)]);
            assert_ne!(
                desktop.view_camera.unwrap(),
                prior,
                "held navigation canceled by stale native camera, shift={shift}"
            );
            let state = display.lock().unwrap();
            assert_eq!(
                state.requests.presented.as_ref().unwrap().camera,
                presented.camera
            );
            assert!(state.requests.pending_scene.is_none());
        }
        assert_ne!(desktop.view_camera.unwrap(), initial);
        assert_eq!(scene.snapshot().unwrap(), before);
    }
}

#[test]
fn actual_notice_and_panel_bounds_preserve_primary_cleanup_cause_and_controls() {
    for [width, height] in [[320., 420.], [480., 420.], [736., 560.], [1024., 560.]] {
        let ctx = egui::Context::default();
        theme::apply(&ctx);
        let mut style = (*ctx.style_of(egui::Theme::Dark)).clone();
        for font in style.text_styles.values_mut() {
            font.size *= 1.2;
        }
        ctx.set_style_of(egui::Theme::Dark, style);
        let display = Arc::new(Mutex::new(RunState::default()));
        {
            let mut s = display.lock().unwrap();
            s.ready = true;
            s.finished = true;
            s.cleanup_failed = true;
            s.error = Some(Notice::new(
                "PRIMARY_EXECUTION_FAILURE",
                "recover the owned participants",
                "complete original cause",
            ));
            s.document_error = Some("LATER_DOCUMENT_REJECTION".into());
        }
        let world = Arc::new(Mutex::new(Handle::fixture(display.clone())));
        let mut desktop = Desktop::new(world, display.clone());
        desktop.inspector_open = true;
        desktop.console_open = true;
        desktop.follow = Some(follow::Target {
            epoch: scene::SceneEpoch {
                execution: "document".into(),
                model: crate::mujoco::ModelIdentity([0; 32]),
                generation: 0,
            },
            body: 1,
            label: "a very long actual robot name that must remain bounded and accessible".into(),
        });
        desktop.message = Some("LATER_UI_REJECTION".into());
        desktop.texture = Some(ctx.load_texture(
            "notice-world",
            egui::ColorImage::filled([64, 48], egui::Color32::BLACK),
            Default::default(),
        ));
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, height));
        let mut text = Vec::new();
        for _ in 0..4 {
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(screen),
                    ..Default::default()
                },
                |ui| desktop.draw(ui),
            );
            text = out
                .shapes
                .iter()
                .filter_map(|s| {
                    if let egui::Shape::Text(t) = &s.shape {
                        Some(t.galley.text().to_owned())
                    } else {
                        None
                    }
                })
                .collect();
            out.textures_delta.clear();
        }
        let canvas = desktop.last_viewport.unwrap();
        for (index, rect) in desktop.captured.iter().enumerate() {
            assert!(
                if width < 600. && index == 2 {
                    screen.contains_rect(*rect) && rect.top() >= canvas.bottom()
                } else {
                    canvas.contains_rect(*rect)
                },
                "{width}: overlay outside canvas {rect:?}"
            );
        }
        let panel = desktop.captured[2];
        let notice = desktop.captured[3];
        assert!(
            !panel.intersects(notice),
            "{width}: panel {panel:?} overlaps notice {notice:?}"
        );
        for (index, a) in desktop.captured.iter().enumerate() {
            for b in &desktop.captured[index + 1..] {
                assert!(
                    !a.intersects(*b),
                    "{width}: overlapping actual surfaces {a:?} {b:?}"
                );
            }
        }
        let upper = desktop.captured[0].top().min(desktop.captured[1].top());
        assert!(upper - notice.bottom() >= 12. - 0.1);
        assert!(text.iter().any(|t| t == "PRIMARY_EXECUTION_FAILURE"));
        assert!(!text.iter().any(|t| t == "LATER_UI_REJECTION"));
        desktop.message = None;
        desktop.console_open = true;
        for _ in 0..3 {
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(screen),
                    ..Default::default()
                },
                |ui| desktop.draw(ui),
            );
            out.textures_delta.clear();
        }
        assert!(display.lock().unwrap().cleanup_failed);
        assert_eq!(
            display.lock().unwrap().error.as_ref().unwrap().primary,
            "PRIMARY_EXECUTION_FAILURE"
        );
    }
}

#[test]
fn actual_header_exposes_scene_identity_and_follow_does_not_overlap_contextual_panel() {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let display = Arc::new(Mutex::new(RunState::default()));
    {
        let mut state = display.lock().unwrap();
        state.ready = true;
        state.finished = true;
        state.document.scene_name = "Actual selected environment.xml".into();
    }
    let world = Arc::new(Mutex::new(Handle::fixture(display.clone())));
    let mut desktop = Desktop::new(world, display);
    desktop.inspector_open = true;
    desktop.follow = Some(follow::Target {
        epoch: scene::SceneEpoch {
            execution: "document".into(),
            model: crate::mujoco::ModelIdentity([0; 32]),
            generation: 0,
        },
        body: 1,
        label: "robot-rover".into(),
    });
    desktop.texture = Some(ctx.load_texture(
        "design-review",
        egui::ColorImage::filled([64, 48], egui::Color32::BLACK),
        Default::default(),
    ));
    let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(320., 560.));
    let mut texts = Vec::new();
    for _ in 0..4 {
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        texts = out
            .shapes
            .iter()
            .filter_map(|s| {
                if let egui::Shape::Text(t) = &s.shape {
                    Some(t.galley.text().to_owned())
                } else {
                    None
                }
            })
            .collect();
        out.textures_delta.clear();
    }
    assert!(
        !desktop.last_follow.unwrap().intersects(desktop.captured[2]),
        "Follow occupies the contextual panel lane"
    );
    assert!(
        texts.iter().any(|t| t.contains("Actual selected")),
        "actual scene identity absent from header: {texts:?}"
    );
}

#[test]
fn import_modal_traps_live_focus_bound_scaled_content_and_restore_invoker() {
    for width in [320., 480., 736.] {
        for import in [true] {
            let ctx = egui::Context::default();
            theme::apply(&ctx);
            let mut style = (*ctx.style_of(egui::Theme::Dark)).clone();
            for font in style.text_styles.values_mut() {
                font.size *= 1.2;
            }
            ctx.set_style_of(egui::Theme::Dark, style);
            let display = Arc::new(Mutex::new(RunState::default()));
            {
                let mut s = display.lock().unwrap();
                s.ready = true;
                s.finished = true;
                s.document
                    .attachments
                    .push(("robot_mount".into(), [0.; 3], [1., 0., 0., 0.]));
            }
            let world = Arc::new(Mutex::new(Handle::fixture(display.clone())));
            let mut desktop = Desktop::new(world, display);
            desktop.texture = Some(ctx.load_texture(
                "modal-world",
                egui::ColorImage::filled([64, 48], egui::Color32::BLACK),
                Default::default(),
            ));
            let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, 560.));
            let draw = |desktop: &mut Desktop, events| {
                let mut out = ctx.run_ui(
                    egui::RawInput {
                        screen_rect: Some(screen),
                        events,
                        ..Default::default()
                    },
                    |ui| desktop.draw(ui),
                );
                out.textures_delta.clear();
            };
            for _ in 0..3 {
                draw(&mut desktop, vec![]);
            }
            let invoker = ctx
                .data(|d| {
                    d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(
                        egui::Id::new("control-probes"),
                    )
                })
                .unwrap()
                .iter()
                .find(|p| p.0 == "Help")
                .unwrap()
                .2;
            // Let the focus request land before the modal opens, as real
            // invocations do, so the modal records its restore target.
            ctx.memory_mut(|m| m.request_focus(invoker));
            draw(&mut desktop, vec![]);
            if import {
                desktop.import_open = true;
                desktop.import_build = Some(
                    "/selected real prepared build directory with a long name and spaces".into(),
                );
                desktop.import_target = "robot_mount".into();
            }
            for _ in 0..4 {
                draw(&mut desktop, vec![]);
            }
            let rect = ctx
                .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("modal-bounds")))
                .unwrap();
            assert!(screen.contains_rect(rect), "{width}: modal clips {rect:?}");
            let ids = ctx
                .data(|d| d.get_temp::<Vec<egui::Id>>(egui::Id::new("modal-focus-probes")))
                .unwrap();
            assert!(ids.len() >= 3);
            assert!(ids.contains(&ctx.memory(|m| m.focused()).unwrap()));
            for shift in [false, false, false, true, true, true] {
                draw(
                    &mut desktop,
                    vec![egui::Event::Key {
                        key: egui::Key::Tab,
                        physical_key: None,
                        pressed: true,
                        repeat: false,
                        modifiers: egui::Modifiers {
                            shift,
                            ..Default::default()
                        },
                    }],
                );
                assert!(
                    ids.contains(&ctx.memory(|m| m.focused()).unwrap()),
                    "focus escaped actual modal"
                );
            }
            draw(
                &mut desktop,
                vec![egui::Event::Key {
                    key: egui::Key::Escape,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: Default::default(),
                }],
            );
            assert!(!desktop.import_open);
            // The modal owner disabled the background while closing; focus
            // restore retries on the first frame the invoker is enabled
            // again (see ModalFocus::restore).
            draw(&mut desktop, vec![]);
            assert_eq!(ctx.memory(|m| m.focused()), Some(invoker));
        }
    }
}

#[test]
#[cfg(native_qualification)]
fn explicit_native_outline_selection_reveals_inspector_once_and_respects_deliberate_close() {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let handle = Handle::start(None).unwrap();
    let display = handle.display.clone();
    wait(&display, |s| s.ready && s.frame.is_some());
    let world = Arc::new(Mutex::new(handle));
    let mut desktop = Desktop::new(world.clone(), display.clone());
    let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(320., 560.));
    let draw = |desktop: &mut Desktop, events| {
        ctx.data_mut(|d| d.remove::<Vec<(usize, egui::Rect)>>(egui::Id::new("outline-row-probes")));
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                events,
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        out.textures_delta.clear();
    };
    for _ in 0..3 {
        draw(&mut desktop, vec![]);
    }
    assert!(!desktop.inspector_open);
    for _ in 0..2 {
        desktop.outline_open = true;
        for _ in 0..3 {
            draw(&mut desktop, vec![]);
        }
        let point = ctx
            .data(|d| d.get_temp::<Vec<(usize, egui::Rect)>>(egui::Id::new("outline-row-probes")))
            .unwrap()
            .iter()
            .find(|r| r.0 == 0)
            .unwrap()
            .1
            .center();
        draw(
            &mut desktop,
            vec![
                egui::Event::PointerMoved(point),
                egui::Event::PointerButton {
                    pos: point,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: Default::default(),
                },
            ],
        );
        draw(
            &mut desktop,
            vec![egui::Event::PointerButton {
                pos: point,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: Default::default(),
            }],
        );
        wait(&display, |s| {
            s.scene_result.is_some() && s.selection.as_ref().is_some_and(|x| x.body == 0)
        });
        draw(&mut desktop, vec![]);
        assert!(
            desktop.inspector_open && !desktop.outline_open,
            "new explicit native selection failed to reveal narrow inspector"
        );
        let point = ctx
            .data(|d| {
                d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                    "control-probes",
                ))
            })
            .unwrap()
            .iter()
            .rev()
            .find(|r| r.0 == "Inspector")
            .unwrap()
            .1
            .center();
        draw(
            &mut desktop,
            vec![
                egui::Event::PointerMoved(point),
                egui::Event::PointerButton {
                    pos: point,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: Default::default(),
                },
            ],
        );
        draw(
            &mut desktop,
            vec![egui::Event::PointerButton {
                pos: point,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: Default::default(),
            }],
        );
        assert!(!desktop.inspector_open);
        for _ in 0..4 {
            draw(&mut desktop, vec![]);
        }
        assert!(
            !desktop.inspector_open,
            "observer refresh reopened deliberately closed inspector"
        );
    }
    world.lock().unwrap().shutdown().unwrap();
}

#[test]
fn actual_pacing_response_has_keyboard_focus_current_value_and_prerequisite_help() {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let display = Arc::new(Mutex::new(RunState::default()));
    let world = Arc::new(Mutex::new(Handle::fixture(display.clone())));
    let mut desktop = Desktop::new(world, display.clone());
    let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(736., 560.));
    let draw = |desktop: &mut Desktop| {
        let mut output = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        output.textures_delta.clear();
        output
    };
    for _ in 0..3 {
        draw(&mut desktop);
    }
    let (rect, id) = ctx
        .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("pacing-probe")))
        .unwrap();
    assert!(screen.contains_rect(rect));
    {
        let mut state = display.lock().unwrap();
        state.finished = false;
        state.ready = true;
    }
    ctx.memory_mut(|m| m.request_focus(id));
    draw(&mut desktop);
    assert!(ctx.memory(|m| m.has_focus(id)));
    desktop.help_open = true;
    let mut texts = Vec::new();
    for _ in 0..3 {
        let output = draw(&mut desktop);
        texts = output
            .shapes
            .iter()
            .filter_map(|s| {
                if let egui::Shape::Text(t) = &s.shape {
                    Some(t.galley.text().to_owned())
                } else {
                    None
                }
            })
            .collect();
    }
    assert!(
        texts
            .iter()
            .any(|t| t.contains("pacing") && t.contains("Start")),
        "missing keyboard-reachable pacing prerequisite: {texts:?}"
    );
}

#[test]
#[cfg(native_qualification)]
fn actual_place_button_commits_native_rover_and_reveals_context_once() {
    let build = std::path::PathBuf::from(std::env::var("PHOXAL_QUALIFICATION_BUILD").unwrap());
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let handle = Handle::start(None).unwrap();
    let display = handle.display.clone();
    wait(&display, |s| s.ready && s.frame.is_some());
    handle
        .edit(Edit::Import {
            build: build.clone(),
            target: "robot_mount".into(),
            position: [0.; 3],
            quaternion: [1., 0., 0., 0.],
        })
        .unwrap();
    wait(&display, |s| {
        !s.document.busy && s.document.candidate.is_some()
    });
    let world = Arc::new(Mutex::new(handle));
    let mut desktop = Desktop::new(world.clone(), display.clone());
    desktop.import_open = true;
    desktop.import_build = Some(build);
    desktop.import_target = "robot_mount".into();
    desktop.import_position = [0.; 3];
    desktop.import_quaternion = [1., 0., 0., 0.];
    desktop.outline_open = true;
    let draw = |desktop: &mut Desktop, events| {
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(480., 720.),
                )),
                events,
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        out.textures_delta.clear();
    };
    for _ in 0..4 {
        draw(&mut desktop, vec![]);
    }
    // Scrolling belongs to the sheet: the placement backdrop owns the rest
    // of the background, so the wheel must hover the sheet itself.
    let sheet = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("modal-bounds")))
        .expect("sheet bounds before scroll");
    draw(
        &mut desktop,
        vec![
            egui::Event::PointerMoved(sheet.center()),
            egui::Event::MouseWheel {
                unit: egui::MouseWheelUnit::Point,
                phase: egui::TouchPhase::Move,
                delta: egui::vec2(0., -600.),
                modifiers: Default::default(),
            },
        ],
    );
    for _ in 0..4 {
        draw(&mut desktop, vec![]);
    }
    let point = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("place-probe")))
        .unwrap()
        .center();
    for pressed in [true, false] {
        draw(
            &mut desktop,
            vec![
                egui::Event::PointerMoved(point),
                egui::Event::PointerButton {
                    pos: point,
                    button: egui::PointerButton::Primary,
                    pressed,
                    modifiers: Default::default(),
                },
            ],
        );
    }
    let candidate_details = display
        .lock()
        .unwrap()
        .document
        .candidate
        .as_ref()
        .map(|c| (c.target.clone(), c.position, c.quaternion));
    assert!(
        desktop.placement_pending,
        "Place did not dispatch: point={point:?}, message={:?}, candidate={:?}",
        desktop.message, candidate_details
    );
    wait(&display, |s| !s.document.busy && s.document.robot.is_some());
    draw(&mut desktop, vec![]);
    assert!(desktop.inspector_open && !desktop.outline_open && !desktop.import_open);
    assert!(display.lock().unwrap().selection.is_some());
    desktop.inspector_open = false;
    for _ in 0..4 {
        draw(&mut desktop, vec![]);
    }
    assert!(!desktop.inspector_open);
    world.lock().unwrap().shutdown().unwrap();
}

#[test]
#[cfg(native_qualification)]
fn actual_run_pause_selection_uses_the_current_execution_epoch() {
    // Regression for the stale document presentation epoch: after a real
    // rover Place and Run/Pause cycle, the authoritative execution viewport
    // must supersede the pre-run document presentation so fresh selection
    // through the same queue path is accepted. Two full cycles prove the
    // fence survives repeated generation transitions.
    let build = std::path::PathBuf::from(std::env::var("PHOXAL_QUALIFICATION_BUILD").unwrap());
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let handle = Handle::start(None).unwrap();
    let display = handle.display.clone();
    wait(&display, |s| s.ready && s.frame.is_some());
    handle
        .edit(Edit::Import {
            build: build.clone(),
            target: "robot_mount".into(),
            position: [0.; 3],
            quaternion: [1., 0., 0., 0.],
        })
        .unwrap();
    wait(&display, |s| {
        !s.document.busy && s.document.candidate.is_some()
    });
    handle.edit(Edit::Place).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_some());
    let world = Arc::new(Mutex::new(handle));
    let mut desktop = Desktop::new(world.clone(), display.clone());
    let draw = |desktop: &mut Desktop, events| {
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1024., 688.),
                )),
                events,
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        let pairs: Vec<(String, egui::Rect)> = out
            .shapes
            .iter()
            .filter_map(|shape| {
                if let egui::Shape::Text(text) = &shape.shape {
                    Some((
                        text.galley.text().to_owned(),
                        shape.shape.visual_bounding_rect(),
                    ))
                } else {
                    None
                }
            })
            .collect();
        ctx.data_mut(|d| d.insert_temp(egui::Id::new("text-probes"), pairs));
        out.textures_delta.clear();
    };
    for cycle in 1..=2u64 {
        handle_send_run(&world);
        wait(&display, |s| s.running);
        draw(&mut desktop, vec![]);
        handle_send_pause(&world);
        wait(&display, |s| !s.running && s.ready);
        // Let the desktop publish the execution frame and present it.
        for _ in 0..6 {
            draw(&mut desktop, vec![]);
        }
        let generation = display
            .lock()
            .unwrap()
            .requests
            .presented
            .as_ref()
            .map(|view| view.identity.epoch.generation)
            .unwrap_or_else(|| panic!("cycle {cycle}: no presented view after run/pause"));
        // The presented identity must belong to the post-run execution
        // world, never the pre-run document presentation (generation 0).
        assert!(
            generation >= 1,
            "cycle {cycle}: presented generation {generation} is the stale document epoch"
        );
        // A fresh selection through the exact outline queue path is
        // accepted against the current execution presentation.
        {
            let mut state = display.lock().unwrap();
            let view = state.requests.presented.clone().unwrap();
            state.requests.pending_scene = Some(scene::SceneRequest {
                frame: view.identity.clone(),
                action: scene::SceneAction::SelectBody(2),
            });
        }
        wait(&display, |s| s.selection.is_some());
        // With the real presented snapshot, Properties shows the
        // world-frame body-origin units on the live narrow context panel.
        desktop.inspector_open = true;
        let mut narrow_out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(480., 688.),
                )),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        let texts: Vec<String> = narrow_out
            .shapes
            .iter()
            .filter_map(|shape| {
                if let egui::Shape::Text(text) = &shape.shape {
                    Some(text.galley.text().to_owned())
                } else {
                    None
                }
            })
            .collect();
        narrow_out.textures_delta.clear();
        assert!(
            texts
                .iter()
                .any(|t| t.contains("world frame · body origin")),
            "cycle {cycle}: properties units missing: {texts:?}"
        );
        desktop.inspector_open = false;
        {
            let mut state = display.lock().unwrap();
            let view = state.requests.presented.clone().unwrap();
            state.requests.pending_scene = Some(scene::SceneRequest {
                frame: view.identity.clone(),
                action: scene::SceneAction::Clear,
            });
        }
        wait(&display, |s| s.selection.is_none());
    }
    world.lock().unwrap().shutdown().unwrap();
}

#[cfg(native_qualification)]
fn world_send(handle: &Handle, command: Command) {
    handle.send(command).unwrap();
}
#[cfg(any(native_qualification, external_source_qualification))]
fn handle_send_run(world: &Arc<Mutex<Handle>>) {
    world.lock().unwrap().send(Command::Run).unwrap();
}
#[cfg(any(native_qualification, external_source_qualification))]
fn handle_send_pause(world: &Arc<Mutex<Handle>>) {
    world.lock().unwrap().send(Command::Pause).unwrap();
}

#[test]
#[cfg(native_qualification)]
fn import_position_is_world_site_position_not_attachment_local_offset() {
    // The placement contract writes the supplied vector directly to the
    // selected world-level site position: a translated and rotated
    // attachment with input [1,0,0] places the robot root at [1,0,0.21],
    // never as an offset in the attachment's own rotated frame.
    let bundle = crate::bundle::BundleFacts::load(&std::path::PathBuf::from(
        std::env::var("PHOXAL_QUALIFICATION_BUILD").unwrap(),
    ))
    .unwrap();
    let xml = r#"<mujoco><option timestep="0.002"/><worldbody><geom type="plane" size="10 10 0.1"/><site name="shifted" pos="5 2 0" quat="0.7071067811865476 0 0 0.7071067811865476"/></worldbody></mujoco>"#;
    let closed = crate::mujoco::ClosedModel::from_xml(xml).unwrap();
    let baseline =
        crate::composition::compose_closed(closed.clone(), &bundle, "shifted", None).unwrap();
    let moved = crate::composition::compose_closed(
        closed,
        &bundle,
        "shifted",
        Some(([1., 0., 0.], [1., 0., 0., 0.])),
    )
    .unwrap();
    let baseline_snapshot = crate::mujoco::Workspace::new(&baseline)
        .unwrap()
        .snapshot()
        .unwrap();
    let moved_snapshot = crate::mujoco::Workspace::new(&moved)
        .unwrap()
        .snapshot()
        .unwrap();
    let baseline_pos = baseline_snapshot.body_positions()[baseline.robot_root().unwrap()];
    let moved_pos = moved_snapshot.body_positions()[moved.robot_root().unwrap()];
    eprintln!(
        "PLACEMENT_CONTRACT baseline root {baseline_pos:?}; input [1,0,0] root {moved_pos:?}"
    );
    assert!((baseline_pos[0] - 5.).abs() < 1e-9 && (baseline_pos[1] - 2.).abs() < 1e-9);
    assert!(
        (moved_pos[0] - 1.).abs() < 1e-9 && moved_pos[1].abs() < 1e-9,
        "position unexpectedly treated as an attachment-local offset"
    );
}

#[test]
#[cfg(native_qualification)]
fn real_rover_wide_inspector_settles_and_displays_components_and_scopes() {
    // Settled real-data geometry: after placement and selection, the wide
    // panel's compact rows show all three Position components, the units
    // heading and both action scopes together, measured through each
    // text shape's actual clipping rectangle.
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let handle = Handle::start(None).unwrap();
    let display = handle.display.clone();
    wait(&display, |s| s.ready && s.frame.is_some());
    handle
        .edit(Edit::Import {
            build: std::path::PathBuf::from(std::env::var("PHOXAL_QUALIFICATION_BUILD").unwrap()),
            target: "robot_mount".into(),
            position: [1., 0., 0.],
            quaternion: [1., 0., 0., 0.],
        })
        .unwrap();
    wait(&display, |s| {
        !s.document.busy && s.document.candidate.is_some()
    });
    handle.edit(Edit::Place).unwrap();
    wait(&display, |s| {
        !s.document.busy && s.document.robot.is_some() && s.frame.is_some()
    });
    let world = Arc::new(Mutex::new(handle));
    let mut desktop = Desktop::new(world.clone(), display.clone());
    let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1024., 688.));
    let draw = |desktop: &mut Desktop| {
        ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(screen),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        )
    };
    for _ in 0..6 {
        let mut output = draw(&mut desktop);
        output.textures_delta.clear();
        std::thread::sleep(Duration::from_millis(30));
    }
    let root = display.lock().unwrap().robot_root.unwrap();
    {
        let mut state = display.lock().unwrap();
        let view = state.requests.presented.clone().unwrap();
        state.requests.pending_scene = Some(scene::SceneRequest {
            frame: view.identity.clone(),
            action: scene::SceneAction::SelectBody(root),
        });
    }
    wait(&display, |s| {
        s.selection.as_ref().is_some_and(|hit| hit.body == root)
    });
    desktop.inspector_open = true;
    let mut seen_units = false;
    let mut seen_scopes = false;
    for frame in 0..10 {
        let mut out = draw(&mut desktop);
        let panel = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("inspector-panel-probe")))
            .unwrap();
        let visible: Vec<String> = out
            .shapes
            .iter()
            .filter_map(|shape| {
                if let egui::Shape::Text(text) = &shape.shape {
                    let bounds = shape.shape.visual_bounding_rect();
                    let visible = bounds.intersect(shape.clip_rect).intersect(panel);
                    if visible.is_positive() {
                        return Some(text.galley.text().to_owned());
                    }
                }
                None
            })
            .collect();
        out.textures_delta.clear();
        if frame >= 3 {
            assert!(
                panel.is_positive() && panel.width() >= 200. && screen.contains_rect(panel),
                "settled panel invalid: {panel:?}"
            );
            seen_units |= visible
                .iter()
                .any(|text| text.contains("world frame · body origin"));
            seen_scopes |= visible.iter().any(|text| text == "Camera")
                && visible.iter().any(|text| text == "Selection");
            assert!(
                ["X", "Y", "Z"]
                    .iter()
                    .all(|axis| visible.iter().any(|text| text == axis)),
                "settled panel cannot show the three compact Position components together: {visible:?}"
            );
        }
        std::thread::sleep(Duration::from_millis(30));
    }
    assert!(seen_units, "units heading never visible");
    assert!(
        seen_scopes,
        "camera/selection scopes never visible together"
    );
    world.lock().unwrap().shutdown().unwrap();
}

#[test]
#[cfg(native_qualification)]
fn real_panel_actions_bind_follow_and_remove_and_undo_through_owner() {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let handle = Handle::start(None).unwrap();
    let display = handle.display.clone();
    wait(&display, |s| s.ready && s.frame.is_some());
    handle
        .edit(Edit::Import {
            build: std::path::PathBuf::from(std::env::var("PHOXAL_QUALIFICATION_BUILD").unwrap()),
            target: "robot_mount".into(),
            position: [1., 0., 0.],
            quaternion: [1., 0., 0., 0.],
        })
        .unwrap();
    wait(&display, |s| {
        !s.document.busy && s.document.candidate.is_some()
    });
    handle.edit(Edit::Place).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_some());
    let world = Arc::new(Mutex::new(handle));
    let mut desktop = Desktop::new(world.clone(), display.clone());
    let draw = |desktop: &mut Desktop, events| {
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1024., 688.),
                )),
                events,
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        out.textures_delta.clear();
    };
    for _ in 0..6 {
        draw(&mut desktop, vec![]);
    }
    let root = display.lock().unwrap().robot_root.unwrap();
    {
        let mut state = display.lock().unwrap();
        let view = state.requests.presented.clone().unwrap();
        state.requests.pending_scene = Some(scene::SceneRequest {
            frame: view.identity.clone(),
            action: scene::SceneAction::SelectBody(root),
        });
    }
    wait(&display, |s| s.selection.is_some());
    desktop.inspector_open = true;
    for _ in 0..5 {
        draw(&mut desktop, vec![]);
    }
    // Follow binds the real robot target through the panel button.
    // The narrow context panel reliably lays out in this owner context;
    // drive the panel actions there and settle before reading probes.
    let draw_narrow = |desktop: &mut Desktop, events| {
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(480., 688.),
                )),
                events,
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        let pairs: Vec<(String, egui::Rect)> = out
            .shapes
            .iter()
            .filter_map(|shape| {
                if let egui::Shape::Text(text) = &shape.shape {
                    let bounds = shape.shape.visual_bounding_rect();
                    let clip = shape.clip_rect.expand(0.5);
                    clip.contains_rect(bounds)
                        .then_some((text.galley.text().to_owned(), bounds))
                } else {
                    None
                }
            })
            .collect();
        ctx.data_mut(|d| d.insert_temp(egui::Id::new("text-probes"), pairs));
        out.textures_delta.clear();
    };
    for _ in 0..6 {
        draw_narrow(&mut desktop, vec![]);
        std::thread::sleep(Duration::from_millis(20));
    }
    let clear_exists = ctx
        .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("clear-action-id-probe")))
        .is_some();
    let follow_probe = ctx.data(|d| {
        d.get_temp::<(egui::Rect, egui::Id, bool)>(egui::Id::new("follow-action-id-probe"))
    });
    let _ = clear_exists;
    let (mut follow_rect, follow_id, mut follow_enabled): (egui::Rect, egui::Id, bool) =
        follow_probe.expect("follow probe");
    // Scroll the narrow context panel until the Follow action is actually
    // on screen inside the strip, then interact with it.
    let visible = |rect: egui::Rect| rect.top() > 420. && rect.bottom() < 640.;
    for _ in 0..24 {
        if visible(follow_rect) {
            break;
        }
        draw_narrow(
            &mut desktop,
            vec![
                egui::Event::PointerMoved(egui::pos2(240., 560.)),
                egui::Event::MouseWheel {
                    unit: egui::MouseWheelUnit::Point,
                    phase: egui::TouchPhase::Move,
                    delta: egui::vec2(0., -20.),
                    modifiers: Default::default(),
                },
            ],
        );
        draw_narrow(&mut desktop, vec![]);
        if let Some(probe) = ctx.data(|d| {
            d.get_temp::<(egui::Rect, egui::Id, bool)>(egui::Id::new("follow-action-id-probe"))
        }) {
            follow_rect = probe.0;
            follow_enabled = probe.2;
        }
    }
    assert!(
        visible(follow_rect),
        "Follow action never reached the visible strip: {follow_rect:?}"
    );
    assert!(follow_enabled, "panel Follow disabled with presentation");
    let click = |desktop: &mut Desktop, point: egui::Pos2| {
        draw_narrow(
            desktop,
            vec![
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
            ],
        );
        draw_narrow(desktop, vec![]);
    };
    click(&mut desktop, follow_rect.center());
    let bound = desktop.follow.as_ref().map(|t| t.label.clone());
    assert_eq!(
        bound.as_deref(),
        Some("robot-rover"),
        "panel Follow did not bind the real target"
    );
    // Stop following releases it through the same owner: scroll it into
    // the visible strip first, then click its actual visible bounds.
    let _ = follow_id;
    let stop_point = {
        let mut found: Option<egui::Pos2> = None;
        for _ in 0..24 {
            let probes = ctx
                .data(|d| d.get_temp::<Vec<(String, egui::Rect)>>(egui::Id::new("text-probes")))
                .unwrap_or_default();
            if let Some((_, rect)) = probes
                .into_iter()
                .find(|(text, _)| text.contains("Stop following"))
            {
                found = Some(rect.center());
                break;
            }
            draw_narrow(
                &mut desktop,
                vec![
                    egui::Event::PointerMoved(egui::pos2(240., 560.)),
                    egui::Event::MouseWheel {
                        unit: egui::MouseWheelUnit::Point,
                        phase: egui::TouchPhase::Move,
                        delta: egui::vec2(0., -20.),
                        modifiers: Default::default(),
                    },
                ],
            );
            draw_narrow(&mut desktop, vec![]);
        }
        found.expect("Stop following never reached the visible strip")
    };
    click(&mut desktop, stop_point);
    assert!(desktop.follow.is_none(), "Stop following did not release");
    // Remove from scene removes the whole robot and creates the undo
    // checkpoint through the real owner.
    let (_, _, remove_enabled): (egui::Rect, egui::Id, bool) = ctx
        .data(|d| d.get_temp(egui::Id::new("remove-action-id-probe")))
        .expect("remove probe");
    assert!(remove_enabled, "Remove disabled while stopped");
    let scroll_to = |desktop: &mut Desktop, probe_id: &str| -> egui::Pos2 {
        let mut point: Option<egui::Pos2> = None;
        for _ in 0..36 {
            if let Some((rect, _, _)) = ctx
                .data(|d| d.get_temp::<(egui::Rect, egui::Id, bool)>(egui::Id::new(probe_id)))
                .filter(|(rect, _, _)| rect.top() > 420. && rect.bottom() < 640.)
            {
                point = Some(rect.center());
                break;
            }
            draw_narrow(
                desktop,
                vec![
                    egui::Event::PointerMoved(egui::pos2(240., 560.)),
                    egui::Event::MouseWheel {
                        unit: egui::MouseWheelUnit::Point,
                        phase: egui::TouchPhase::Move,
                        delta: egui::vec2(0., -20.),
                        modifiers: Default::default(),
                    },
                ],
            );
            draw_narrow(desktop, vec![]);
        }
        point.unwrap_or_else(|| panic!("{probe_id} never reached the visible strip"))
    };
    let remove_point = scroll_to(&mut desktop, "remove-action-id-probe");
    click(&mut desktop, remove_point);
    wait(&display, |s| {
        s.document.robot.is_none() && s.document.removed_robot.is_some()
    });
    // Undo restores it through the same panel owner.
    for _ in 0..5 {
        draw(&mut desktop, vec![]);
    }
    let undo_point = scroll_to(&mut desktop, "undo-action-id-probe");
    click(&mut desktop, undo_point);
    wait(&display, |s| {
        s.document.robot.is_some() && s.document.removed_robot.is_none()
    });
    world.lock().unwrap().shutdown().unwrap();
}

#[test]
fn header_scene_menu_is_content_sized_with_flexible_space_and_padded_import() {
    for width in [320., 736., 1024.] {
        let (ctx, _desktop, draw) = chrome_harness(width, 560.);
        for _ in 0..3 {
            draw.borrow_mut()(vec![]);
        }
        let menu = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("document-menu-probe")))
            .unwrap();
        let import = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("import-button-probe")))
            .unwrap();
        assert!(
            menu.width() < 220.,
            "{width}: scene menu is not content-sized {menu:?}"
        );
        assert!(
            menu.right() + 30. <= import.left(),
            "{width}: no flexible space between header groups {menu:?} {import:?}"
        );
        assert!((import.height() - 30.).abs() < 0.1, "{width}: {import:?}");
        assert!(import.width() > 40., "{width}: cramped import {import:?}");
    }
}

#[test]
fn persistent_footer_reserves_viewport_and_carries_mode_sim_console() {
    for width in [320., 480., 736., 1024.] {
        let (ctx, desktop, draw) = chrome_harness(width, 560.);
        for _ in 0..3 {
            draw.borrow_mut()(vec![]);
        }
        let footer = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("footer-probe")))
            .unwrap();
        let canvas = desktop.borrow().last_viewport.unwrap();
        assert!(
            footer.top() >= canvas.bottom() - 0.5,
            "{width}: footer overlaps the world {footer:?} {canvas:?}"
        );
        assert!(
            (footer.width() - width).abs() < 1.,
            "{width}: footer is not full-width {footer:?}"
        );
        assert!(canvas.height() > 560. * 0.5, "{width}: {canvas:?}");
        assert!(
            ctx.data(
                |d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe"))
            )
            .is_some(),
            "{width}: console action missing from the footer"
        );
        let texts = draw.borrow_mut()(vec![]);
        assert!(
            texts.iter().any(|t| t.contains("Navigate · camera only")),
            "{width}: footer mode text absent"
        );
        assert!(
            texts.iter().any(|t| t.contains("Sim 0:00 · 0.00x")),
            "{width}: footer must show clock time and realtime factor: {texts:?}"
        );
        desktop.borrow_mut().console_open = true;
        for _ in 0..2 {
            draw.borrow_mut()(vec![]);
        }
        let with_console = desktop.borrow().last_viewport.unwrap();
        assert!(
            with_console.height() < canvas.height(),
            "{width}: console drawer does not reserve viewport space"
        );
    }
}

#[test]
fn candidate_preview_samples_the_full_nonuniform_image_beside_the_sheet() {
    // Representative source ratios across all required widths at normal
    // and enlarged text: the painted candidate keeps its own aspect,
    // full-image UVs, whole-frame containment, sheet exclusion, and the
    // measured label band stays inside the frame and clear of the image.
    for scale in [1.0f32, 1.4] {
        for (source_w, source_h) in [(4u32, 2u32), (4, 3), (3, 4)] {
            for width in [1024., 736., 480., 320.] {
                let (ctx, desktop, draw) = chrome_harness(width, 560.);
                if scale != 1.0 {
                    ctx.all_styles_mut(|style| {
                        let body = style.text_styles.get(&egui::TextStyle::Body).cloned();
                        if let Some(font) = body {
                            style.text_styles.insert(
                                egui::TextStyle::Body,
                                egui::FontId::proportional(font.size * scale),
                            );
                        }
                    });
                }
                let world = desktop.borrow().world.clone();
                {
                    let world = world.lock().unwrap();
                    let mut state = world.display.lock().unwrap();
                    // A deliberately nonuniform frame: correct full-image UVs
                    // must map all quadrants; corner sampling cannot.
                    let mut rgb = vec![0u8; (source_w * source_h * 3) as usize];
                    for y in 0..source_h {
                        for x in 0..source_w {
                            let i = ((y * source_w + x) * 3) as usize;
                            rgb[i] = if x < source_w / 2 { 255 } else { 0 };
                            rgb[i + 1] = if y < source_h / 2 { 255 } else { 0 };
                            rgb[i + 2] = 128;
                        }
                    }
                    let image = crate::mujoco::RenderedCamera::from_parts(
                        [source_w as usize, source_h as usize],
                        rgb.into_boxed_slice(),
                        vec![0f32; (source_w * source_h) as usize].into_boxed_slice(),
                    );
                    state.document.candidate = Some(crate::app::world::CandidateView {
                        identity: "robot-rover · robot_mount\nManifest 0000\nModel 1111".into(),
                        build: std::path::PathBuf::from("/prepared/robot"),
                        target: "robot_mount".into(),
                        position: [0.; 3],
                        quaternion: [1., 0., 0., 0.],
                        image: std::sync::Arc::new(image),
                    });
                }
                {
                    let mut desktop = desktop.borrow_mut();
                    desktop.import_open = true;
                    desktop.import_build = Some(std::path::PathBuf::from("/prepared/robot"));
                    desktop.import_target = "robot_mount".into();
                    desktop.import_position = [0.; 3];
                    desktop.import_quaternion = [1., 0., 0., 0.];
                }
                for _ in 0..4 {
                    draw.borrow_mut()(vec![]);
                }
                let shapes: Vec<egui::Shape> = ctx
                    .data(|d| d.get_temp(egui::Id::new("shape-probes")))
                    .expect("shapes captured");
                // Select the candidate's own mesh by its texture id; the
                // confirmed world texture and icon meshes are irrelevant to
                // this defect. The historical Rect::ZERO UV bound produced
                // identical corner-texel UVs on exactly this mesh, so the UV
                // assertion fails against that defect.
                let candidate_texture = desktop
                    .borrow()
                    .candidate_texture
                    .as_ref()
                    .map(|(_, handle)| handle.id())
                    .expect("candidate texture loaded");
                let candidate_mesh = shapes
                    .iter()
                    .find_map(|shape| match shape {
                        egui::Shape::Mesh(mesh) if mesh.texture_id == candidate_texture => {
                            Some(mesh)
                        }
                        _ => None,
                    })
                    .expect("candidate preview mesh painted");
                let (mut min_u, mut min_v) = (f32::INFINITY, f32::INFINITY);
                let (mut max_u, mut max_v) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
                for vertex in &candidate_mesh.vertices {
                    min_u = min_u.min(vertex.uv.x);
                    max_u = max_u.max(vertex.uv.x);
                    min_v = min_v.min(vertex.uv.y);
                    max_v = max_v.max(vertex.uv.y);
                }
                assert!(
                    (min_u - 0.).abs() < 1e-3
                        && (max_u - 1.).abs() < 1e-3
                        && (min_v - 0.).abs() < 1e-3
                        && (max_v - 1.).abs() < 1e-3,
                    "candidate UVs must span the whole normalized texture, got u [{min_u:.3},{max_u:.3}] v [{min_v:.3},{max_v:.3}]"
                );
                // The painted image keeps its own source aspect inside the
                // fitted frame chrome.
                let mut min_pos = egui::Pos2::new(f32::INFINITY, f32::INFINITY);
                let mut max_pos = egui::Pos2::new(f32::NEG_INFINITY, f32::NEG_INFINITY);
                for vertex in &candidate_mesh.vertices {
                    min_pos.x = min_pos.x.min(vertex.pos.x);
                    max_pos.x = max_pos.x.max(vertex.pos.x);
                    min_pos.y = min_pos.y.min(vertex.pos.y);
                    max_pos.y = max_pos.y.max(vertex.pos.y);
                }
                let painted_aspect = (max_pos.x - min_pos.x) / (max_pos.y - min_pos.y).max(1e-3);
                let source_aspect = source_w as f32 / source_h as f32;
                assert!(
                    (painted_aspect - source_aspect).abs() / source_aspect < 0.02,
                    "{width}x{source_h} source {source_w}:{source_h}: painted aspect {painted_aspect:.6} distorted"
                );
                // The framed preview stays out of the placement sheet's column.
                let sheet = ctx
                    .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("modal-bounds")))
                    .expect("sheet bounds");
                let canvas = desktop.borrow().last_viewport.unwrap();
                let mut frame: Option<egui::Rect> = None;
                for shape in &shapes {
                    if let egui::Shape::Rect(rect) = shape
                        && rect.fill == egui::Color32::from_black_alpha(210)
                    {
                        frame = Some(rect.rect);
                    }
                }
                if let Some(frame) = frame {
                    assert!(
                        !frame.intersects(sheet),
                        "{width} {source_w}:{source_h} x{scale}: candidate frame overlaps the sheet: {frame:?} vs {sheet:?}"
                    );
                    assert!(
                        canvas.contains_rect(frame),
                        "{width} {source_w}:{source_h} x{scale}: frame outside canvas {frame:?} (canvas {canvas:?})"
                    );
                    // The measured label band: inside the frame width and
                    // clear of the painted image.
                    let texts: Vec<(String, egui::Rect)> = ctx
                        .data(|d| d.get_temp(egui::Id::new("text-probes")))
                        .unwrap();
                    let label = texts
                        .iter()
                        .find(|(text, _)| text.contains("Candidate preview"))
                        .map(|(_, rect)| *rect)
                        .unwrap_or_else(|| panic!("{width} x{scale}: label missing"));
                    assert!(
                        frame.contains_rect(label),
                        "{width} {source_w}:{source_h} x{scale}: label {label:?} escapes frame {frame:?}"
                    );
                    let mut min_pos = egui::Pos2::new(f32::INFINITY, f32::INFINITY);
                    let mut max_pos = egui::Pos2::new(f32::NEG_INFINITY, f32::NEG_INFINITY);
                    for vertex in &candidate_mesh.vertices {
                        min_pos.x = min_pos.x.min(vertex.pos.x);
                        max_pos.x = max_pos.x.max(vertex.pos.x);
                        min_pos.y = min_pos.y.min(vertex.pos.y);
                        max_pos.y = max_pos.y.max(vertex.pos.y);
                    }
                    let image_bounds = egui::Rect::from_min_max(min_pos, max_pos);
                    assert!(
                        !label.intersects(image_bounds),
                        "{width} {source_w}:{source_h} x{scale}: label {label:?} overlaps image {image_bounds:?}"
                    );
                } else {
                    panic!("{width}: candidate frame not painted");
                }
            }
        }
    }
}

#[test]
fn accessible_background_activation_is_gated_by_the_modal_owner() {
    let (ctx, desktop, draw) = chrome_harness(1024., 560.);
    desktop.borrow_mut().import_open = true;
    desktop.borrow_mut().import_build = Some(std::path::PathBuf::from("/prepared/robot"));
    desktop.borrow_mut().import_target = "robot_mount".into();
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let (_, toggle_id) = ctx
        .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe")))
        .unwrap();
    // A native element action during placement must not activate Console.
    draw.borrow_mut()(vec![egui::Event::AccessKitActionRequest(
        egui::accesskit::ActionRequest {
            action: egui::accesskit::Action::Click,
            target_tree: egui::accesskit::TreeId::ROOT,
            target_node: egui::accesskit::NodeId(toggle_id.value()),
            data: None,
        },
    )]);
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    assert!(
        !desktop.borrow().console_open,
        "accessible activation must be gated during placement"
    );
    // Fresh input reaches the footer again after cancellation.
    draw.borrow_mut()(vec![egui::Event::Key {
        key: egui::Key::Escape,
        physical_key: None,
        pressed: true,
        repeat: false,
        modifiers: Default::default(),
    }]);
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    let (rect, _) = ctx
        .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe")))
        .unwrap();
    let point = rect.center();
    draw.borrow_mut()(vec![
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
    ]);
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    assert!(
        desktop.borrow().console_open,
        "fresh input must reach the console toggle after placement closes"
    );
}

#[test]
fn console_opened_outside_import_keeps_the_footer_at_the_window_bottom() {
    let (ctx, desktop, draw) = chrome_harness(736., 560.);
    desktop.borrow_mut().console_open = true;
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let texts = draw.borrow_mut()(vec![]);
    assert!(texts.iter().any(|t| t.contains("Diagnostics")));
    let (toggle_rect, _toggle_id) = ctx
        .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe")))
        .unwrap();
    // The footer row must reach the window bottom: no abandoned strip.
    assert!(
        (560. - toggle_rect.bottom()).abs() < 12.,
        "footer floats above the window bottom: {toggle_rect:?}"
    );
    desktop.borrow_mut().outline_open = true;
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let (narrow_rect, _) = ctx
        .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe")))
        .unwrap();
    assert!(
        (560. - narrow_rect.bottom()).abs() < 12.,
        "narrow panel plus console must keep the footer anchored: {narrow_rect:?}"
    );
}

#[test]
fn keyboard_focus_paints_last_over_selected_and_hovered_fills() {
    let (ctx, desktop, draw) = chrome_harness(736., 560.);
    desktop.borrow_mut().console_open = true;
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let (rect, id) = ctx
        .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe")))
        .unwrap();
    ctx.memory_mut(|memory| memory.request_focus(id));
    draw.borrow_mut()(vec![]);
    let shapes: Vec<egui::Shape> = ctx
        .data(|d| d.get_temp(egui::Id::new("shape-probes")))
        .expect("shapes captured");
    let mut focus_index = None;
    let mut last_covering_fill = None;
    for (index, shape) in shapes.iter().enumerate() {
        if let egui::Shape::Rect(rect_shape) = shape {
            let covers = rect_shape.rect.contains_rect(rect);
            if covers
                && rect_shape.stroke.color == egui::Color32::WHITE
                && rect_shape.stroke.width >= 2.
            {
                focus_index = Some(index);
            }
            if covers && rect_shape.fill != egui::Color32::TRANSPARENT {
                last_covering_fill = Some(index);
            }
        }
    }
    let focus_index = focus_index.expect("focused console outline painted");
    assert!(
        focus_index > last_covering_fill.unwrap_or(0),
        "focus outline must paint after selected/hover fills: focus {focus_index}, last fill {last_covering_fill:?}"
    );
    // Inspector tabs keep a visible outline when focused: the widget style
    // provides the white active stroke instead of forcing none.
    let visuals = ctx.style_of(egui::Theme::Dark).visuals.clone();
    assert_eq!(visuals.widgets.active.bg_stroke.color, egui::Color32::WHITE);
}

#[test]
fn panel_actions_are_gated_under_every_modal_owner() {
    for (owner, expect_outline_open) in [("import", true), ("help", true)] {
        let (ctx, desktop, draw) = chrome_harness(1024., 560.);
        {
            let mut desktop = desktop.borrow_mut();
            desktop.outline_open = true;
            match owner {
                "import" => {
                    desktop.import_open = true;
                    desktop.import_build = Some(std::path::PathBuf::from("/prepared/robot"));
                    desktop.import_target = "robot_mount".into();
                }
                _ => desktop.help_open = true,
            }
        }
        for _ in 0..3 {
            draw.borrow_mut()(vec![]);
        }
        // The detached outline panel's Close action must be disabled for
        // accessible activation while the modal owns input.
        let close = ctx
            .data(|d| {
                d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                    "control-probes",
                ))
            })
            .unwrap()
            .into_iter()
            .find(|probe| probe.0 == "Close outline");
        let Some((_, rect, id, _, _, enabled)) = close else {
            panic!("{owner}: outline Close not probed");
        };
        assert!(!enabled, "{owner}: outline Close must be disabled");
        draw.borrow_mut()(vec![egui::Event::AccessKitActionRequest(
            egui::accesskit::ActionRequest {
                action: egui::accesskit::Action::Click,
                target_tree: egui::accesskit::TreeId::ROOT,
                target_node: egui::accesskit::NodeId(id.value()),
                data: None,
            },
        )]);
        for _ in 0..2 {
            draw.borrow_mut()(vec![]);
        }
        assert!(
            desktop.borrow().outline_open == expect_outline_open,
            "{owner}: AccessKit activation closed a background panel"
        );
        // After dismissal, panel actions accept fresh input again.
        draw.borrow_mut()(vec![egui::Event::Key {
            key: egui::Key::Escape,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: Default::default(),
        }]);
        for _ in 0..2 {
            draw.borrow_mut()(vec![]);
        }
        let point = rect.center();
        draw.borrow_mut()(vec![
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
        ]);
        for _ in 0..2 {
            draw.borrow_mut()(vec![]);
        }
        assert!(
            !desktop.borrow().outline_open,
            "{owner}: fresh input must close the outline after dismissal"
        );
    }
}

#[test]
fn focused_inspector_tabs_retain_a_visible_outline_on_the_live_widget() {
    let (ctx, desktop, draw) = chrome_harness(1024., 560.);
    desktop.borrow_mut().inspector_open = true;
    {
        let world = desktop.borrow().world.clone();
        let world = world.lock().unwrap();
        world.display.lock().unwrap().selection = Some(crate::mujoco::NativeSelection {
            body: 0,
            geom: None,
            point: [0.; 3],
        });
    }
    for _ in 0..4 {
        draw.borrow_mut()(vec![]);
    }
    for tab in ["Properties", "Motion"] {
        let all_tabs = ctx
            .data(|d| {
                d.get_temp::<Vec<(&'static str, egui::Rect, egui::Id)>>(egui::Id::new("tab-probes"))
            })
            .unwrap();
        let (_, tab_rect, tab_id) = all_tabs
            .iter()
            .rev()
            .find(|(label, _, _)| *label == tab)
            .copied()
            .unwrap_or_else(|| panic!("{tab} tab not probed"));
        // Focus the real live widget.
        ctx.memory_mut(|memory| memory.request_focus(tab_id));
        // Settle with real elapsed time so the Area's fade-in completes
        // and the outline reaches full white, as it does in a live UI.
        for _ in 0..8 {
            draw.borrow_mut()(vec![]);
            std::thread::sleep(std::time::Duration::from_millis(15));
        }
        let shapes: Vec<egui::Shape> = ctx
            .data(|d| d.get_temp(egui::Id::new("shape-probes")))
            .expect("shapes captured");
        // egui Areas fade in; without advancing wall time the fade alpha
        // settles partway, so accept clearly visible white (full RGB with
        // at least ~63 percent alpha) at the settled frame width.
        let outlined = shapes.iter().any(|shape| {
            if let egui::Shape::Rect(rect) = shape {
                let color = rect.stroke.color;
                color.r() == 255
                    && color.g() == 255
                    && color.b() == 255
                    && color.a() >= 160
                    && rect.stroke.width >= 2.
                    && rect.rect.intersects(tab_rect)
            } else {
                false
            }
        });
        assert!(
            outlined,
            "{tab} tab must show a distinct white focus outline"
        );
    }
}

#[test]
fn stale_candidate_place_keeps_readable_disabled_colors() {
    let (ctx, desktop, draw) = chrome_harness(1024., 560.);
    let world = desktop.borrow().world.clone();
    {
        let world = world.lock().unwrap();
        let mut state = world.display.lock().unwrap();
        let image = crate::mujoco::RenderedCamera::from_parts(
            [4, 3],
            vec![120u8; 36].into_boxed_slice(),
            vec![0f32; 12].into_boxed_slice(),
        );
        // The candidate differs from the current placement controls, so
        // Place stays disabled (stale) exactly as after an edit.
        state.document.candidate = Some(crate::app::world::CandidateView {
            identity: "robot-rover · other_mount\\nManifest 0000\\nModel 1111".into(),
            build: std::path::PathBuf::from("/prepared/robot"),
            target: "other_mount".into(),
            position: [1., 2., 3.],
            quaternion: [1., 0., 0., 0.],
            image: std::sync::Arc::new(image),
        });
    }
    {
        let mut desktop = desktop.borrow_mut();
        desktop.import_open = true;
        desktop.import_build = Some(std::path::PathBuf::from("/prepared/robot"));
        desktop.import_target = "robot_mount".into();
        desktop.import_position = [0.; 3];
        desktop.import_quaternion = [1., 0., 0., 0.];
    }
    for _ in 0..4 {
        draw.borrow_mut()(vec![]);
    }
    // Scroll the sheet so the deep Place action is on screen.
    let sheet = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("modal-bounds")))
        .expect("sheet bounds");
    draw.borrow_mut()(vec![
        egui::Event::PointerMoved(sheet.center()),
        egui::Event::MouseWheel {
            unit: egui::MouseWheelUnit::Point,
            phase: egui::TouchPhase::Move,
            delta: egui::vec2(0., -600.),
            modifiers: Default::default(),
        },
    ]);
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    // Let the Area and widget background animations settle with real
    // elapsed time so fill colors reach their final values.
    for _ in 0..26 {
        draw.borrow_mut()(vec![]);
        std::thread::sleep(std::time::Duration::from_millis(15));
    }
    let place = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("place-probe")))
        .expect("place probe");
    let shapes: Vec<egui::Shape> = ctx
        .data(|d| d.get_temp(egui::Id::new("shape-probes")))
        .expect("shapes captured");
    let texts: Vec<(String, egui::Rect)> = ctx
        .data(|d| d.get_temp(egui::Id::new("text-probes")))
        .unwrap();
    assert!(
        texts.iter().any(|(text, _)| text.contains("Place robot")),
        "Place label present"
    );
    // A stale Place keeps the raised disabled fill (painted at the theme's
    // disabled alpha); the dark accent treatment belongs to the enabled
    // primary only.
    let alpha = 0.75f32;
    let expected = theme::RAISED_FILL;
    let raised = shapes.iter().any(|shape| {
        if let egui::Shape::Rect(rect) = shape
            && rect.rect.contains_rect(place)
        {
            let fill = rect.fill;
            (f32::from(fill.r()) - f32::from(expected.r()) * alpha).abs() <= 2.
                && (f32::from(fill.g()) - f32::from(expected.g()) * alpha).abs() <= 2.
                && (f32::from(fill.b()) - f32::from(expected.b()) * alpha).abs() <= 2.
        } else {
            false
        }
    });
    assert!(raised, "stale Place must keep the raised disabled fill");
}

#[test]
fn footer_clock_sits_content_width_beside_the_console_toggle() {
    for width in [1024., 736., 480., 320.] {
        let (ctx, _desktop, draw) = chrome_harness(width, 560.);
        for _ in 0..3 {
            draw.borrow_mut()(vec![]);
        }
        let (toggle, _) = ctx
            .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe")))
            .unwrap();
        let texts: Vec<(String, egui::Rect)> = ctx
            .data(|d| d.get_temp(egui::Id::new("text-probes")))
            .unwrap();
        let (_, clock) = texts
            .iter()
            .find(|(text, _)| text.contains("Sim 0:00"))
            .unwrap_or_else(|| panic!("{width}: clock missing {texts:?}"));
        assert!(
            (toggle.left() - clock.right() - 14.).abs() < 4.,
            "{width}: clock not adjacent to Console: clock {clock:?} toggle {toggle:?}"
        );
    }
}

#[test]
fn narrow_import_sheet_leaves_the_footer_visible_and_below_the_sheet() {
    for width in [480., 320.] {
        let (ctx, desktop, draw) = chrome_harness(width, 688.);
        {
            let mut desktop = desktop.borrow_mut();
            desktop.import_open = true;
            desktop.import_build = Some(std::path::PathBuf::from("/prepared/robot"));
            desktop.import_target = "robot_mount".into();
            desktop.import_position = [0.; 3];
            desktop.import_quaternion = [1., 0., 0., 0.];
        }
        for _ in 0..4 {
            draw.borrow_mut()(vec![]);
        }
        let sheet = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("modal-bounds")))
            .expect("sheet bounds");
        let (footer, _) = ctx
            .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe")))
            .unwrap();
        // The footer row stays anchored at the window bottom...
        assert!(
            (688. - footer.bottom()).abs() < 12.,
            "{width}: footer floats at {footer:?}"
        );
        // ...and the sheet never covers it with positive area.
        let overlap = sheet.intersect(footer);
        assert!(
            overlap.is_negative() || overlap.area() <= 0.01,
            "{width}: sheet {sheet:?} covers footer {footer:?} (overlap {overlap:?})"
        );
        // Keyboard focus reaches the live sheet controls while the
        // background is owned by the modal.
        for _ in 0..8 {
            draw.borrow_mut()(vec![egui::Event::Key {
                key: egui::Key::Tab,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: Default::default(),
            }]);
        }
        let sheet_ids = ctx
            .data(|d| d.get_temp::<Vec<egui::Id>>(egui::Id::new("modal-focus-probes")))
            .unwrap_or_default();
        let focused = ctx.memory(|m| m.focused());
        assert!(
            focused.is_some_and(|id| sheet_ids.contains(&id)),
            "{width}: keyboard focus never reached the sheet (focused {focused:?} of {} sheet ids)",
            sheet_ids.len()
        );
    }
}

#[test]
fn header_scene_identity_shows_fully_at_wide_widths() {
    let (ctx, _desktop, draw) = chrome_harness(1024., 560.);
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    // The full default identity stays visible with the 16pt chevron and
    // measured padding; only genuine space pressure truncates.
    let texts: Vec<(String, egui::Rect)> = ctx
        .data(|d| d.get_temp(egui::Id::new("text-probes")))
        .unwrap();
    assert!(
        texts.iter().any(|(text, _)| text == "Default scene"),
        "full identity truncated at 1024: {texts:?}"
    );
    let menu = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("document-menu-probe")))
        .unwrap();
    let (_, label) = texts
        .iter()
        .find(|(text, _)| text == "Default scene")
        .unwrap();
    assert!(
        menu.width() >= label.width() + icons::GLYPH + 12.,
        "menu {menu:?} does not fit label {label:?} plus glyph"
    );
}

#[test]
fn inspector_groups_show_verified_identity_units_scopes_and_undo() {
    let (ctx, desktop, draw) = chrome_harness(1024., 560.);
    {
        let world = desktop.borrow().world.clone();
        let world = world.lock().unwrap();
        let mut state = world.display.lock().unwrap();
        // A robot member selection with a presented snapshot: verified
        // membership, world-frame units and both scopes must be visible.
        state.document.robot = Some("robot-rover".into());
        state.robot_root = Some(1);
        state.bodies = std::sync::Arc::from(
            vec![
                crate::mujoco::NativeBody {
                    id: 0,
                    name: "world".into(),
                    parent: None,
                    mobility: crate::mujoco::BodyMobility::Fixed,
                },
                crate::mujoco::NativeBody {
                    id: 1,
                    name: "robot-rover__base_link".into(),
                    parent: Some(0),
                    mobility: crate::mujoco::BodyMobility::FreeJoint,
                },
            ]
            .into_boxed_slice(),
        );
        state.selection = Some(crate::mujoco::NativeSelection {
            body: 1,
            geom: None,
            point: [0.; 3],
        });
        state.document.removed_robot = None;
    }
    desktop.borrow_mut().inspector_open = true;
    for _ in 0..4 {
        draw.borrow_mut()(vec![]);
    }
    let capture = |ctx: &egui::Context| -> Vec<String> {
        let texts: Vec<(String, egui::Rect)> = ctx
            .data(|d| d.get_temp(egui::Id::new("text-probes")))
            .unwrap();
        texts.into_iter().map(|(text, _)| text).collect()
    };
    // The panel scrolls; the header stays at the top, and the Robot in
    // scene scope needs scrolling into view.
    let top: Vec<String> = capture(&ctx);
    let top_has = |needle: &str| top.iter().any(|t| t.contains(needle));
    let panel = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("inspector-panel-probe")))
        .expect("inspector panel");
    draw.borrow_mut()(vec![
        egui::Event::PointerMoved(panel.center()),
        egui::Event::MouseWheel {
            unit: egui::MouseWheelUnit::Point,
            phase: egui::TouchPhase::Move,
            delta: egui::vec2(0., -400.),
            modifiers: Default::default(),
        },
    ]);
    for _ in 0..4 {
        draw.borrow_mut()(vec![]);
    }
    let texts = capture(&ctx);
    let has = |needle: &str| texts.iter().any(|t| t.contains(needle));
    // Verified identity: friendly name plus membership, never an inferred
    // role or a duplicate Source row.
    assert!(top_has("base_link"), "friendly identifier missing: {top:?}");
    assert!(
        top_has("Part of robot-rover"),
        "membership missing: {top:?}"
    );
    assert!(
        !top.iter().any(|t| t.contains("Source ")),
        "duplicate source row retained: {top:?}"
    );
    assert!(top_has("Camera"), "camera scope missing: {top:?}");
    // World-frame units on Properties are gated on the native presentation
    // in the owner; they are asserted in the qualification lane where a
    // real presented snapshot exists.
    // Camera and Selection scopes with their explanations.
    assert!(
        top_has("Target: robot-rover"),
        "camera target missing: {top:?}"
    );
    assert!(top_has("Follow robot"), "follow action missing: {top:?}");
    assert!(
        has("Clears inspection; keeps the object in the scene"),
        "selection scope wording missing: {texts:?}"
    );
    // Robot in scene scope with whole-robot removal wording and Undo
    // through the same panel owner.
    assert!(has("Robot in scene"), "robot scope missing: {texts:?}");
    assert!(
        has("Removes the whole robot; source files are kept"),
        "removal scope missing: {texts:?}"
    );
    assert!(
        !has("Undo removal"),
        "undo shown without a checkpoint: {texts:?}"
    );
    // Diagnostics stays collapsed: the native id/mobility line must not
    // leak into the visible panel while the section is closed.
    assert!(
        !has("Native body 1 · Free joint"),
        "diagnostics leaked while collapsed: {texts:?}"
    );
    // With the removed identity projection, Undo appears with its truthful
    // scope wording through the same panel.
    {
        let world = desktop.borrow().world.clone();
        let world = world.lock().unwrap();
        let mut state = world.display.lock().unwrap();
        state.document.robot = None;
        state.document.removed_robot = Some("robot-rover".into());
    }
    let panel2 = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("inspector-panel-probe")))
        .expect("inspector panel");
    draw.borrow_mut()(vec![
        egui::Event::PointerMoved(panel2.center()),
        egui::Event::MouseWheel {
            unit: egui::MouseWheelUnit::Point,
            phase: egui::TouchPhase::Move,
            delta: egui::vec2(0., -400.),
            modifiers: Default::default(),
        },
    ]);
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let texts_after: Vec<String> = {
        let captured: Vec<(String, egui::Rect)> = ctx
            .data(|d| d.get_temp(egui::Id::new("text-probes")))
            .unwrap();
        captured.into_iter().map(|(text, _)| text).collect()
    };
    assert!(
        texts_after.iter().any(|t| t.contains("Undo removal")),
        "undo missing with a checkpoint: {texts_after:?}"
    );
    assert!(
        texts_after
            .iter()
            .any(|t| t.contains("Restores robot-rover at its original placement")),
        "undo scope wording missing: {texts_after:?}"
    );
    // Diagnostics opens with the actual word order (native id before
    // mobility) rather than the obsolete reversed phrasing.
    let texts_now: Vec<(String, egui::Rect)> = ctx
        .data(|d| d.get_temp(egui::Id::new("text-probes")))
        .unwrap();
    let (_, diag) = texts_now
        .iter()
        .find(|(text, _)| text == "Diagnostics")
        .unwrap();
    let point = diag.center();
    draw.borrow_mut()(vec![
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
    ]);
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let opened: Vec<String> = {
        let captured: Vec<(String, egui::Rect)> = ctx
            .data(|d| d.get_temp(egui::Id::new("text-probes")))
            .unwrap();
        captured.into_iter().map(|(text, _)| text).collect()
    };
    assert!(
        opened
            .iter()
            .any(|t| t.contains("Native body 1 · Free joint")),
        "diagnostics word order wrong when opened: {opened:?}"
    );
}

#[test]
fn inspector_motion_components_render_from_live_data_at_focused_widths() {
    for (width, scale) in [(1024., 1.0f32), (320., 1.0), (1024., 1.4), (320., 1.4)] {
        let (ctx, desktop, draw) = chrome_harness(width, 560.);
        if scale != 1.0 {
            ctx.all_styles_mut(|style| {
                let body = style.text_styles.get(&egui::TextStyle::Body).cloned();
                if let Some(font) = body {
                    style.text_styles.insert(
                        egui::TextStyle::Body,
                        egui::FontId::proportional(font.size * scale),
                    );
                }
            });
        }
        let xml = r#"<mujoco><worldbody><geom type="plane" size="10 10 0.1"/><body name="robot-rover__base_link"><freejoint/><geom type="sphere" size="0.2"/></body></worldbody></mujoco>"#;
        let model = crate::mujoco::Model::from_xml(xml.as_bytes()).unwrap();
        let workspace = crate::mujoco::Workspace::new(&model).unwrap();
        let snapshot = workspace.snapshot().unwrap();
        let camera = workspace.default_view_camera();
        {
            let world = desktop.borrow().world.clone();
            let world = world.lock().unwrap();
            let mut state = world.display.lock().unwrap();
            state.document.robot = Some("robot-rover".into());
            state.robot_root = Some(1);
            state.bodies = std::sync::Arc::from(
                vec![
                    crate::mujoco::NativeBody {
                        id: 0,
                        name: "world".into(),
                        parent: None,
                        mobility: crate::mujoco::BodyMobility::Fixed,
                    },
                    crate::mujoco::NativeBody {
                        id: 1,
                        name: "robot-rover__base_link".into(),
                        parent: Some(0),
                        mobility: crate::mujoco::BodyMobility::FreeJoint,
                    },
                ]
                .into_boxed_slice(),
            );
            state.selection = Some(crate::mujoco::NativeSelection {
                body: 1,
                geom: None,
                point: [0.; 3],
            });
            state.requests.presented =
                Some(std::sync::Arc::new(crate::app::scene::PresentedView {
                    identity: crate::app::scene::FrameIdentity {
                        epoch: crate::app::scene::SceneEpoch {
                            execution: "chrome".into(),
                            model: model.identity(),
                            generation: 0,
                        },
                        serial: 1,
                    },
                    camera,
                    resolution: [4, 3],
                    snapshot,
                }));
        }
        desktop.borrow_mut().inspector_open = true;
        for _ in 0..5 {
            draw.borrow_mut()(vec![]);
        }
        // Open the Motion tab through the real tab button.
        let tabs = ctx
            .data(|d| {
                d.get_temp::<Vec<(&'static str, egui::Rect, egui::Id)>>(egui::Id::new("tab-probes"))
            })
            .unwrap();
        let (_, motion_rect, _) = tabs
            .iter()
            .rev()
            .find(|(label, _, _)| *label == "Motion")
            .copied()
            .unwrap();
        draw.borrow_mut()(vec![
            egui::Event::PointerMoved(motion_rect.center()),
            egui::Event::PointerButton {
                pos: motion_rect.center(),
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: Default::default(),
            },
            egui::Event::PointerButton {
                pos: motion_rect.center(),
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: Default::default(),
            },
        ]);
        for _ in 0..4 {
            draw.borrow_mut()(vec![]);
        }
        // Progressive 20pt scroll sampling with the clipping-aware probes.
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, 560.));
        let mut seen_orientation = false;
        let mut seen_linear = false;
        let mut seen_angular = false;
        for step in 0..36 {
            if step > 0 {
                let wheel_x = if width < 600. {
                    width * 0.5
                } else {
                    width - 120.
                };
                let wheel_y = if width < 600. { 460. } else { 320. };
                draw.borrow_mut()(vec![
                    egui::Event::PointerMoved(egui::pos2(wheel_x, wheel_y)),
                    egui::Event::MouseWheel {
                        unit: egui::MouseWheelUnit::Point,
                        phase: egui::TouchPhase::Move,
                        delta: egui::vec2(0., -20.),
                        modifiers: Default::default(),
                    },
                ]);
                for _ in 0..2 {
                    draw.borrow_mut()(vec![]);
                }
            }
            let visible: Vec<String> = ctx
                .data(|d| d.get_temp::<Vec<(String, egui::Rect)>>(egui::Id::new("text-probes")))
                .unwrap()
                .into_iter()
                .map(|(text, _)| text)
                .collect();
            let has = |needle: &str| visible.iter().any(|t| t.contains(needle));
            seen_orientation |= has("Orientation · quaternion W/X/Y/Z");
            seen_linear |= has("Linear velocity · m/s");
            seen_angular |= has("Angular velocity · rad/s");
            for (text, rect) in ctx
                .data(|d| d.get_temp::<Vec<(String, egui::Rect)>>(egui::Id::new("text-probes")))
                .unwrap()
            {
                if rect.is_finite() && !screen.contains_rect(rect) {
                    panic!("{width} x{scale}: {text:?} escapes at {rect:?}");
                }
            }
        }
        assert!(
            seen_orientation,
            "{width} x{scale}: Motion orientation never visible"
        );
        assert!(
            seen_linear,
            "{width} x{scale}: Motion linear velocity never visible"
        );
        assert!(
            seen_angular,
            "{width} x{scale}: Motion angular velocity never visible"
        );
    }
}

#[test]
fn inspector_empty_state_keeps_robot_scope_without_tabs_or_duplication() {
    let (ctx, desktop, draw) = chrome_harness(1024., 560.);
    {
        let world = desktop.borrow().world.clone();
        let world = world.lock().unwrap();
        world.display.lock().unwrap().document.robot = Some("robot-rover".into());
    }
    desktop.borrow_mut().inspector_open = true;
    for _ in 0..4 {
        draw.borrow_mut()(vec![]);
    }
    let texts: Vec<String> = {
        let texts: Vec<(String, egui::Rect)> = ctx
            .data(|d| d.get_temp(egui::Id::new("text-probes")))
            .unwrap();
        texts.into_iter().map(|(text, _)| text).collect()
    };
    let has = |needle: &str| texts.iter().any(|t| t.contains(needle));
    assert!(has("Nothing selected"), "empty state missing: {texts:?}");
    assert!(
        has("Click an object in the scene or choose one in Outline"),
        "guidance missing: {texts:?}"
    );
    assert!(!has("Properties"), "tabs shown while empty: {texts:?}");
    // One prompt only, and the robot scope remains available.
    assert_eq!(
        texts.iter().filter(|t| *t == "Nothing selected").count(),
        1,
        "duplicate empty prompts: {texts:?}"
    );
    assert!(has("Robot in scene"), "robot scope missing: {texts:?}");
}

#[test]
fn inspector_structure_stays_visible_and_unclipped_across_widths_text_and_targets() {
    // Complete live-path matrix: real presented snapshot (Properties and
    // Motion components), larger-target preference with measured 30/40
    // response heights, clip-rect visibility, keyboard activation of a
    // panel action, and owned Clear/Follow effects, across
    // 1024/736/480/320 at normal and 1.4x text.
    // A real snapshot whose body 1 carries a position, so the presented
    // Properties rows have live values for the selected member.
    let xml = r#"<mujoco><worldbody><geom type="plane" size="10 10 0.1"/><body name="robot-rover__base_link"><freejoint/><geom type="sphere" size="0.2"/></body></worldbody></mujoco>"#;
    let model = crate::mujoco::Model::from_xml(xml.as_bytes()).unwrap();
    let workspace = crate::mujoco::Workspace::new(&model).unwrap();
    let snapshot = workspace.snapshot().unwrap();
    let camera = workspace.default_view_camera();
    for scale in [1.0f32, 1.4] {
        for larger in [false, true] {
            for width in [1024., 736., 480., 320.] {
                let (ctx, desktop, draw) = chrome_harness(width, 560.);
                if scale != 1.0 {
                    ctx.all_styles_mut(|style| {
                        let body = style.text_styles.get(&egui::TextStyle::Body).cloned();
                        if let Some(font) = body {
                            style.text_styles.insert(
                                egui::TextStyle::Body,
                                egui::FontId::proportional(font.size * scale),
                            );
                        }
                    });
                }
                {
                    let world = desktop.borrow().world.clone();
                    let world = world.lock().unwrap();
                    let mut state = world.display.lock().unwrap();
                    state.document.robot = Some("robot-rover".into());
                    state.robot_root = Some(1);
                    state.bodies = std::sync::Arc::from(
                        vec![
                            crate::mujoco::NativeBody {
                                id: 0,
                                name: "world".into(),
                                parent: None,
                                mobility: crate::mujoco::BodyMobility::Fixed,
                            },
                            crate::mujoco::NativeBody {
                                id: 1,
                                name: "robot-rover__base_link".into(),
                                parent: Some(0),
                                mobility: crate::mujoco::BodyMobility::FreeJoint,
                            },
                        ]
                        .into_boxed_slice(),
                    );
                    state.selection = Some(crate::mujoco::NativeSelection {
                        body: 1,
                        geom: None,
                        point: [0.; 3],
                    });
                    state.requests.presented =
                        Some(std::sync::Arc::new(crate::app::scene::PresentedView {
                            identity: crate::app::scene::FrameIdentity {
                                epoch: crate::app::scene::SceneEpoch {
                                    execution: "chrome".into(),
                                    model: model.identity(),
                                    generation: 0,
                                },
                                serial: 1,
                            },
                            camera,
                            resolution: [4, 3],
                            snapshot: snapshot.clone(),
                        }));
                }
                {
                    let mut desktop = desktop.borrow_mut();
                    desktop.inspector_open = true;
                    desktop.larger_targets = larger;
                }
                for _ in 0..5 {
                    draw.borrow_mut()(vec![]);
                }
                let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, 560.));
                let mut seen_identity = false;
                let mut seen_camera = false;
                let mut seen_follow = false;
                let mut seen_x = false;
                let mut seen_y = false;
                let mut seen_z = false;
                let mut action_rect: Option<egui::Rect> = None;
                // Wide panels also scroll when scaled content exceeds
                // their height; sample progressive offsets there too.
                let scroll_offsets: &[f32] = if width < 600. {
                    &[
                        0.0, -20.0, -40.0, -60.0, -80.0, -100.0, -120.0, -140.0, -160.0, -180.0,
                        -200.0, -220.0, -240.0, -260.0, -280.0, -300.0, -320.0, -340.0, -360.0,
                        -380.0, -400.0, -420.0, -440.0, -460.0, -480.0, -500.0, -520.0, -540.0,
                        -560.0, -580.0, -600.0, -620.0, -640.0, -660.0, -680.0, -700.0, -720.0,
                    ]
                } else {
                    &[0., -40., -80., -120., -160., -200., -240.]
                };
                for (step, delta) in scroll_offsets.iter().enumerate() {
                    if step > 0 {
                        let wheel_x = if width < 600. {
                            width * 0.5
                        } else {
                            width - 120.
                        };
                        let wheel_y = if width < 600. {
                            if step % 2 == 1 { 460. } else { 500. }
                        } else {
                            320.
                        };
                        draw.borrow_mut()(vec![
                            egui::Event::PointerMoved(egui::pos2(wheel_x, wheel_y)),
                            egui::Event::MouseWheel {
                                unit: egui::MouseWheelUnit::Point,
                                phase: egui::TouchPhase::Move,
                                delta: egui::vec2(0., *delta),
                                modifiers: Default::default(),
                            },
                        ]);
                        for _ in 0..3 {
                            draw.borrow_mut()(vec![]);
                        }
                    }

                    // Clip-rect visibility: a text is visible only when its
                    // bounds intersect its own clip rectangle positively.
                    // Text probes carry clip-surviving bounding rects from
                    // the tessellated output; invisible (clipped) text is
                    // culled before reaching them.
                    let visible: Vec<String> = ctx
                        .data(|d| {
                            d.get_temp::<Vec<(String, egui::Rect)>>(egui::Id::new("text-probes"))
                        })
                        .unwrap()
                        .into_iter()
                        .map(|(text, _)| text)
                        .collect();
                    let captured: Vec<(String, egui::Rect)> = ctx
                        .data(|d| d.get_temp(egui::Id::new("text-probes")))
                        .unwrap();
                    let has = |needle: &str| visible.iter().any(|text| text.contains(needle));
                    seen_identity |= has("base_link") && has("Part of robot-rover");
                    // The scope is evidenced by its header or its
                    // always-visible explanation line; exact header
                    // placement at a scroll boundary is not a defect.
                    seen_camera |= has("Camera") || has("Moves the camera only");
                    seen_follow |= has("Follow robot");
                    // A viewport shorter than three rows shows components
                    // across scroll offsets; each must be reachable and
                    // correctly clipped. The wide case asserts them
                    // together in the settled native regression.
                    seen_x |= has("X");
                    seen_y |= has("Y");
                    seen_z |= has("Z");
                    if action_rect.is_none()
                        && let Some((rect, _)) = ctx.data(|d| {
                            d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new(
                                "clear-action-id-probe",
                            ))
                        })
                    {
                        action_rect = Some(rect);
                    }
                    // No text escapes the screen horizontally.
                    for (text, rect) in &captured {
                        if rect.is_finite() {
                            assert!(
                                screen.contains_rect(*rect),
                                "{width} x{scale} larger={larger}: {text:?} escapes at {rect:?}"
                            );
                        }
                    }
                }
                assert!(
                    seen_identity,
                    "{width} x{scale} larger={larger}: identity never visible"
                );
                assert!(
                    seen_camera,
                    "{width} x{scale} larger={larger}: camera scope never reachable"
                );
                assert!(
                    seen_follow,
                    "{width} x{scale} larger={larger}: follow action never reachable"
                );
                assert!(
                    seen_x && seen_y && seen_z,
                    "{width} x{scale} larger={larger}: X/Y/Z values not all reachable (x={seen_x} y={seen_y} z={seen_z})"
                );
                // Larger-target preference produces real 40pt responses
                // (30pt at the normal preference).
                let expected = if larger { 40. } else { 30. };
                let rect =
                    action_rect.unwrap_or_else(|| panic!("{width}: clear action never probed"));
                assert!(
                    (rect.height() - expected).abs() < 6.,
                    "{width} larger={larger}: action target {rect:?} expected ~{expected}"
                );
                // Keyboard activation of the owned Clear action through the
                // same panel button: focus it and press Enter.
                let (_, clear_id) = ctx
                    .data(|d| {
                        d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("clear-action-id-probe"))
                    })
                    .expect("clear id probe");
                ctx.memory_mut(|memory| memory.request_focus(clear_id));
                draw.borrow_mut()(vec![egui::Event::Key {
                    key: egui::Key::Enter,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: Default::default(),
                }]);
                for _ in 0..3 {
                    draw.borrow_mut()(vec![]);
                }
                let cleared = {
                    let world = desktop.borrow().world.clone();
                    let world = world.lock().unwrap();
                    world
                        .display
                        .lock()
                        .unwrap()
                        .requests
                        .pending_scene
                        .as_ref()
                        .is_some_and(|request| matches!(request.action, scene::SceneAction::Clear))
                };
                assert!(
                    cleared,
                    "{width} x{scale} larger={larger}: keyboard Enter did not activate Clear selection"
                );
            }
        }
    }
}

#[test]
fn follow_without_presentation_keeps_identity_but_gates_binding() {
    // A verified robot selection with no presented view: the target stays
    // identified, the binding action is gated with a visible reason, and
    // clicking nothing binds. Reproduced first through production egui.
    let (ctx, desktop, draw) = chrome_harness(1024., 688.);
    {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        let mut state = handle.display.lock().unwrap();
        state.document.robot = Some("robot-rover".into());
        state.robot_root = Some(1);
        state.bodies = std::sync::Arc::from(
            vec![
                crate::mujoco::NativeBody {
                    id: 0,
                    name: "world".into(),
                    parent: None,
                    mobility: crate::mujoco::BodyMobility::Fixed,
                },
                crate::mujoco::NativeBody {
                    id: 1,
                    name: "robot-rover__base_link".into(),
                    parent: Some(0),
                    mobility: crate::mujoco::BodyMobility::FreeJoint,
                },
            ]
            .into_boxed_slice(),
        );
        state.selection = Some(crate::mujoco::NativeSelection {
            body: 1,
            geom: None,
            point: [0.; 3],
        });
        assert!(state.requests.presented.is_none());
    }
    desktop.borrow_mut().inspector_open = true;
    for _ in 0..5 {
        draw.borrow_mut()(vec![]);
    }
    let texts: Vec<(String, egui::Rect)> = ctx
        .data(|d| d.get_temp(egui::Id::new("text-probes")))
        .unwrap();
    let point = texts
        .iter()
        .find(|(text, _)| text == "Follow robot")
        .unwrap()
        .1
        .center();
    draw.borrow_mut()(vec![
        egui::Event::PointerMoved(point),
        egui::Event::PointerButton {
            pos: point,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: Default::default(),
        },
    ]);
    draw.borrow_mut()(vec![egui::Event::PointerButton {
        pos: point,
        button: egui::PointerButton::Primary,
        pressed: false,
        modifiers: Default::default(),
    }]);
    assert!(
        desktop.borrow().follow.is_none(),
        "unexpected target binding without presentation"
    );
    let texts = draw.borrow_mut()(vec![]);
    assert!(
        texts
            .iter()
            .any(|text| text.contains("Waiting for the scene presentation")),
        "no visible missing-presentation prerequisite: {texts:?}"
    );
    // The matching toolbar action is gated by the same prerequisite.
    let toolbar = ctx
        .data(|d| {
            d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                "control-probes",
            ))
        })
        .unwrap()
        .into_iter()
        .find(|probe| probe.0 == "Follow robot")
        .expect("toolbar Follow probe");
    assert!(
        !toolbar.5,
        "toolbar Follow enabled without a presented view"
    );
}

#[test]
fn follow_with_presentation_enables_and_binds_through_both_paths() {
    let (ctx, desktop, draw) = chrome_harness(1024., 688.);
    let xml = r#"<mujoco><worldbody><geom type="plane" size="10 10 0.1"/><body name="robot-rover__base_link"><freejoint/><geom type="sphere" size="0.2"/></body></worldbody></mujoco>"#;
    let model = crate::mujoco::Model::from_xml(xml.as_bytes()).unwrap();
    let workspace = crate::mujoco::Workspace::new(&model).unwrap();
    let snapshot = workspace.snapshot().unwrap();
    let camera = workspace.default_view_camera();
    {
        let world = desktop.borrow().world.clone();
        let world = world.lock().unwrap();
        let mut state = world.display.lock().unwrap();
        state.document.robot = Some("robot-rover".into());
        state.robot_root = Some(1);
        state.bodies = std::sync::Arc::from(
            vec![
                crate::mujoco::NativeBody {
                    id: 0,
                    name: "world".into(),
                    parent: None,
                    mobility: crate::mujoco::BodyMobility::Fixed,
                },
                crate::mujoco::NativeBody {
                    id: 1,
                    name: "robot-rover__base_link".into(),
                    parent: Some(0),
                    mobility: crate::mujoco::BodyMobility::FreeJoint,
                },
            ]
            .into_boxed_slice(),
        );
        state.selection = Some(crate::mujoco::NativeSelection {
            body: 1,
            geom: None,
            point: [0.; 3],
        });
        state.requests.presented = Some(std::sync::Arc::new(crate::app::scene::PresentedView {
            identity: crate::app::scene::FrameIdentity {
                epoch: crate::app::scene::SceneEpoch {
                    execution: "chrome".into(),
                    model: model.identity(),
                    generation: 0,
                },
                serial: 1,
            },
            camera,
            resolution: [4, 3],
            snapshot,
        }));
    }
    desktop.borrow_mut().inspector_open = true;
    for _ in 0..5 {
        draw.borrow_mut()(vec![]);
    }
    // Toolbar enabled with a presented view, and binding succeeds.
    let toolbar = ctx
        .data(|d| {
            d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                "control-probes",
            ))
        })
        .unwrap()
        .into_iter()
        .find(|probe| probe.0 == "Follow robot")
        .expect("toolbar Follow probe");
    assert!(toolbar.5, "toolbar Follow disabled despite presentation");
    draw.borrow_mut()(vec![
        egui::Event::PointerMoved(toolbar.1.center()),
        egui::Event::PointerButton {
            pos: toolbar.1.center(),
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: Default::default(),
        },
        egui::Event::PointerButton {
            pos: toolbar.1.center(),
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        },
    ]);
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let bound = desktop.borrow().follow.as_ref().map(|t| t.label.clone());
    assert_eq!(bound.as_deref(), Some("robot-rover"));
    // An already bound target keeps Stop following regardless of the
    // presentation state.
    {
        let world = desktop.borrow().world.clone();
        let world = world.lock().unwrap();
        world.display.lock().unwrap().requests.presented = None;
    }
    draw.borrow_mut()(vec![]);
    let stop = ctx
        .data(|d| {
            d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                "control-probes",
            ))
        })
        .unwrap()
        .into_iter()
        .find(|probe| probe.0 == "Stop following")
        .expect("toolbar Stop probe");
    assert!(stop.5, "bound Stop following disabled without presentation");
}

#[test]
fn genuine_lucide_assets_rasterize_for_every_mapped_glyph() {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    for icon in icons::Icon::ALL {
        let hex = "B1C2C5";
        let uri = format!("lucide://{slug}-{hex}.svg", slug = icon.slug());
        let bytes = icon
            .svg()
            .replace("currentColor", &format!("#{hex}"))
            .into_bytes();
        ctx.include_bytes(uri.clone(), egui::load::Bytes::from(bytes));
        match ctx.try_load_texture(
            &uri,
            egui::TextureOptions::LINEAR,
            egui::load::SizeHint::Width(48),
        ) {
            Ok(egui::load::TexturePoll::Ready { texture }) => {
                assert!(texture.size.x > 0. && texture.size.y > 0., "{uri} empty");
            }
            _ => panic!("{uri} did not rasterize"),
        }
    }
}

#[test]
fn status_badge_is_content_width_with_minimum_height() {
    for width in [320., 1024.] {
        let (ctx, desktop, draw) = chrome_harness(width, 560.);
        for _ in 0..3 {
            draw.borrow_mut()(vec![]);
        }
        let badge = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("status-badge-probe")))
            .unwrap();
        assert!(
            (badge.height() - 42.).abs() < 0.1,
            "{width}: badge height {badge:?}"
        );
        assert!(badge.width() < 160., "{width}: badge width {badge:?}");
        let canvas = desktop.borrow().last_viewport.unwrap();
        assert!((badge.left() - canvas.left() - 16.).abs() < 0.1);
        assert!((badge.top() - canvas.top() - 16.).abs() < 0.1);
        let texts = draw.borrow_mut()(vec![]);
        assert!(texts.iter().any(|t| t.contains("Ready")));
        assert!(
            !texts.iter().any(|t| t.contains("Wall ")),
            "{width}: detailed timing must stay out of the badge"
        );
    }
}

#[test]
fn status_badge_text_is_contained_and_centered() {
    for width in [320., 1024.] {
        let (ctx, _desktop, draw) = chrome_harness(width, 560.);
        for _ in 0..3 {
            draw.borrow_mut()(vec![]);
        }
        let badge = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("status-badge-probe")))
            .unwrap();
        let pairs: Vec<(String, egui::Rect)> = ctx
            .data(|d| d.get_temp(egui::Id::new("text-probes")))
            .unwrap();
        let (_, text) = pairs
            .iter()
            .find(|(text, _)| text.contains("Ready"))
            .unwrap();
        assert!(
            badge.left() + 6. <= text.left() && text.right() + 6. <= badge.right(),
            "{width}: badge text not contained {badge:?} {text:?}"
        );
        let center_delta = (text.center().y - badge.center().y).abs();
        assert!(
            center_delta < 4.,
            "{width}: badge text off-center by {center_delta}"
        );
    }
}

#[test]
fn import_backdrop_owns_background_until_cancelled() {
    let (ctx, desktop, draw) = chrome_harness(1024., 560.);
    desktop.borrow_mut().import_open = true;
    desktop.borrow_mut().import_build = Some(std::path::PathBuf::from("/prepared/robot"));
    desktop.borrow_mut().import_target = "robot_mount".into();
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let toggle = ctx
        .data(|d| {
            d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                "control-probes",
            ))
        })
        .unwrap()
        .into_iter()
        .find(|probe| probe.0 == "Scene outline")
        .unwrap();
    let point = toggle.1.center();
    draw.borrow_mut()(vec![
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
    ]);
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    assert!(
        !desktop.borrow().outline_open,
        "background header clicks must not reach panels during placement"
    );
    // Escape still cancels placement and returns background ownership.
    draw.borrow_mut()(vec![egui::Event::Key {
        key: egui::Key::Escape,
        physical_key: None,
        pressed: true,
        repeat: false,
        modifiers: Default::default(),
    }]);
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    assert!(
        !desktop.borrow().import_open,
        "Escape must cancel placement"
    );
    draw.borrow_mut()(vec![
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
    ]);
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    assert!(
        desktop.borrow().outline_open,
        "clicks must reach panels again after placement closes"
    );
}

#[test]
fn console_toggle_honors_target_preference_and_activates_from_keyboard() {
    let (ctx, desktop, draw) = chrome_harness(736., 560.);
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let (rect, id) = ctx
        .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe")))
        .unwrap();
    assert!(rect.height() >= 28., "console target too small: {rect:?}");
    ctx.memory_mut(|memory| memory.request_focus(id));
    draw.borrow_mut()(vec![egui::Event::Key {
        key: egui::Key::Enter,
        physical_key: None,
        pressed: true,
        repeat: false,
        modifiers: Default::default(),
    }]);
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    assert!(desktop.borrow().console_open, "keyboard activation failed");
    desktop.borrow_mut().larger_targets = true;
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    let (enlarged, _) = ctx
        .data(|d| d.get_temp::<(egui::Rect, egui::Id)>(egui::Id::new("console-toggle-probe")))
        .unwrap();
    assert!(
        enlarged.height() >= 40.,
        "larger target preference ignored: {enlarged:?}"
    );
}

#[test]
fn import_sheet_scrolls_to_reachable_actions_and_presents_candidate_at_narrow_widths() {
    for width in [480., 320.] {
        let (ctx, desktop, draw) = chrome_harness(width, 720.);
        let world = desktop.borrow().world.clone();
        {
            let world = world.lock().unwrap();
            let mut state = world.display.lock().unwrap();
            let image = crate::mujoco::RenderedCamera::from_parts(
                [320, 240],
                vec![80u8; 320 * 240 * 3].into_boxed_slice(),
                vec![0f32; 320 * 240].into_boxed_slice(),
            );
            state.document.candidate = Some(crate::app::world::CandidateView {
                identity: "robot-rover · robot_mount\nManifest 0000\nModel 1111".into(),
                build: std::path::PathBuf::from("/prepared/robot"),
                target: "robot_mount".into(),
                position: [0.; 3],
                quaternion: [1., 0., 0., 0.],
                image: std::sync::Arc::new(image),
            });
        }
        {
            let mut desktop = desktop.borrow_mut();
            desktop.import_open = true;
            desktop.import_build = Some(std::path::PathBuf::from("/prepared/robot"));
            desktop.import_target = "robot_mount".into();
            desktop.import_position = [0.; 3];
            desktop.import_quaternion = [1., 0., 0., 0.];
        }
        for _ in 0..4 {
            draw.borrow_mut()(vec![]);
        }
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, 720.));
        let sheet = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("modal-bounds")))
            .expect("narrow sheet bounds");
        assert!(
            screen.contains_rect(sheet),
            "narrow sheet exceeds screen {sheet:?}"
        );
        let mut place = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("place-probe")))
            .expect("place probe");
        if !screen.contains_rect(place) {
            draw.borrow_mut()(vec![
                egui::Event::PointerMoved(sheet.center()),
                egui::Event::MouseWheel {
                    unit: egui::MouseWheelUnit::Point,
                    phase: egui::TouchPhase::Move,
                    delta: egui::vec2(0., -600.),
                    modifiers: Default::default(),
                },
            ]);
            for _ in 0..4 {
                draw.borrow_mut()(vec![]);
            }
            place = ctx
                .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("place-probe")))
                .unwrap();
        }
        assert!(
            screen.contains_rect(place),
            "Place stays unreachable after scroll {place:?}"
        );
        // Cancel sits just below Place: scroll to the sheet bottom so both
        // actions are reachable in the same view.
        draw.borrow_mut()(vec![
            egui::Event::PointerMoved(sheet.center()),
            egui::Event::MouseWheel {
                unit: egui::MouseWheelUnit::Point,
                phase: egui::TouchPhase::Move,
                delta: egui::vec2(0., -600.),
                modifiers: Default::default(),
            },
        ]);
        for _ in 0..4 {
            draw.borrow_mut()(vec![]);
        }
        let place = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("place-probe")))
            .unwrap();
        assert!(
            screen.contains_rect(place),
            "{width}: Place response escapes the screen after scroll {place:?}"
        );
        assert!(
            place.max.x <= sheet.max.x + 0.5,
            "{width}: Place response escapes the sheet clip {place:?} vs {sheet:?}"
        );
        // A real Cancel click dismisses the sheet without touching the
        // committed world.
        let cancel_rect = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("cancel-probe")))
            .expect("cancel probe");
        assert!(
            screen.contains_rect(cancel_rect),
            "{width}: Cancel response escapes the screen {cancel_rect:?}"
        );
        let texts = draw.borrow_mut()(vec![]);
        assert!(
            texts
                .iter()
                .any(|t| t.contains("Candidate preview · not placed")),
            "{width}: candidate preview identity missing: {texts:?}"
        );
        let cancel_point = cancel_rect.center();
        draw.borrow_mut()(vec![
            egui::Event::PointerMoved(cancel_point),
            egui::Event::PointerButton {
                pos: cancel_point,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: Default::default(),
            },
            egui::Event::PointerButton {
                pos: cancel_point,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: Default::default(),
            },
        ]);
        for _ in 0..3 {
            draw.borrow_mut()(vec![]);
        }
        assert!(
            !desktop.borrow().import_open,
            "{width}: Cancel click did not dismiss the sheet"
        );
    }
}

#[test]
fn outline_and_inspector_anchor_separate_sides_with_prerequisites() {
    let (ctx, desktop, draw) = chrome_harness(1024., 560.);
    desktop.borrow_mut().outline_open = true;
    desktop.borrow_mut().inspector_open = true;
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let canvas = desktop.borrow().last_viewport.unwrap();
    let outline = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("outline-panel-probe")))
        .unwrap();
    let inspector = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("inspector-panel-probe")))
        .unwrap();
    assert!((outline.left() - canvas.left() - 16.).abs() < 0.1);
    assert!((canvas.right() - inspector.right() - 16.).abs() < 0.1);
    assert!(!outline.intersects(inspector));
    let texts = draw.borrow_mut()(vec![]);
    assert!(
        texts.iter().any(|t| t == "Nothing selected")
            && texts
                .iter()
                .any(|t| t.contains("Click an object in the scene or choose one in Outline")),
        "inspector empty-state text absent: {texts:?}"
    );
    let inspector_button = ctx
        .data(|d| {
            d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                "control-probes",
            ))
        })
        .unwrap()
        .into_iter()
        .find(|probe| probe.0 == "Inspector")
        .unwrap();
    // An open panel's toggle stays enabled so it can close, while a fresh
    // open requires a meaningful selection or a placed robot.
    assert!(inspector_button.5, "an open inspector must remain closable");
    desktop.borrow_mut().inspector_open = false;
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    let closed = ctx
        .data(|d| {
            d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                "control-probes",
            ))
        })
        .unwrap()
        .into_iter()
        .find(|probe| probe.0 == "Inspector")
        .unwrap();
    assert!(!closed.5, "inspector opens without context");
}

#[test]
fn import_sheet_anchors_contextually_and_hides_ordinary_toolbars() {
    for width in [480., 1024.] {
        let (ctx, desktop, draw) = chrome_harness(width, 560.);
        desktop.borrow_mut().import_open = true;
        desktop.borrow_mut().import_build = Some("/selected real prepared build directory".into());
        desktop.borrow_mut().import_target = "robot_mount".into();
        for _ in 0..3 {
            draw.borrow_mut()(vec![]);
        }
        let canvas = desktop.borrow().last_viewport.unwrap();
        let sheet = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("modal-bounds")))
            .unwrap();
        let buttons = ctx
            .data(|d| {
                d.get_temp::<Vec<(String, egui::Rect, egui::Id, bool, bool, bool)>>(egui::Id::new(
                    "control-probes",
                ))
            })
            .unwrap();
        assert!(
            sheet.bottom() <= 560. && sheet.right() <= width + 0.5,
            "{width}: sheet leaves the window {sheet:?}"
        );
        assert!(
            buttons
                .iter()
                .all(|probe| probe.0 != "Run" && probe.0 != "Navigate"),
            "{width}: ordinary viewport toolbars must stay hidden during import"
        );
        if width >= 600. {
            assert!(
                (canvas.right() - sheet.right() - 16.).abs() < 0.5,
                "{width}: wide sheet is not anchored at the right inset {sheet:?}"
            );
            assert!(sheet.left() > canvas.left() + 100.);
        } else {
            assert!(
                sheet.top() >= canvas.bottom() - 0.5,
                "{width}: narrow sheet must reflow below the world {sheet:?} {canvas:?}"
            );
            assert!(canvas.height() > 560. * 0.4, "{width}: {canvas:?}");
        }
    }
}

#[test]
fn outline_label_selects_and_arrow_expands_without_stealing_selection_or_follow() {
    // Reproduced through production owners: the row label (pointer and
    // keyboard) selects the real body, the chevron only expands, and an
    // already-bound Follow target is never retargeted by a selection.
    let (ctx, desktop, draw) = chrome_harness(736., 688.);
    let xml = r#"<mujoco><worldbody><geom type="plane" size="10 10 0.1"/><body name="robot-rover__base_link"><freejoint/><geom type="sphere" size="0.2"/><body name="robot-rover__rotor_link"><geom type="sphere" size="0.1"/></body></body></worldbody></mujoco>"#;
    let model = crate::mujoco::Model::from_xml(xml.as_bytes()).unwrap();
    let workspace = crate::mujoco::Workspace::new(&model).unwrap();
    let camera = workspace.default_view_camera();
    let make_bound = || follow::Target {
        epoch: scene::SceneEpoch {
            execution: "chrome".into(),
            model: model.identity(),
            generation: 0,
        },
        body: 1,
        label: "robot-rover".into(),
    };
    {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        let mut state = handle.display.lock().unwrap();
        let snapshot = workspace.snapshot().unwrap();
        state.requests.presented = Some(std::sync::Arc::new(crate::app::scene::PresentedView {
            identity: crate::app::scene::FrameIdentity {
                epoch: scene::SceneEpoch {
                    execution: "chrome".into(),
                    model: model.identity(),
                    generation: 0,
                },
                serial: 1,
            },
            camera,
            resolution: [4, 3],
            snapshot,
        }));
        state.document.robot = Some("robot-rover".into());
        state.robot_root = Some(1);
        state.bodies = std::sync::Arc::from(
            vec![
                crate::mujoco::NativeBody {
                    id: 0,
                    name: "world".into(),
                    parent: None,
                    mobility: crate::mujoco::BodyMobility::Fixed,
                },
                crate::mujoco::NativeBody {
                    id: 1,
                    name: "robot-rover__base_link".into(),
                    parent: Some(0),
                    mobility: crate::mujoco::BodyMobility::FreeJoint,
                },
                crate::mujoco::NativeBody {
                    id: 2,
                    name: "robot-rover__rotor_link".into(),
                    parent: Some(1),
                    mobility: crate::mujoco::BodyMobility::Fixed,
                },
            ]
            .into_boxed_slice(),
        );
    }
    desktop.borrow_mut().outline_open = true;
    desktop.borrow_mut().follow = Some(make_bound());
    for _ in 0..5 {
        draw.borrow_mut()(vec![]);
    }
    let rows = |ctx: &egui::Context| {
        ctx.data(|d| d.get_temp::<Vec<(usize, egui::Rect)>>(egui::Id::new("outline-row-probes")))
            .unwrap_or_default()
    };
    let arrows = |ctx: &egui::Context| {
        ctx.data(|d| d.get_temp::<Vec<(usize, egui::Rect)>>(egui::Id::new("outline-arrow-probes")))
            .unwrap_or_default()
    };
    let queued = |desktop: &std::cell::RefCell<Desktop>| -> Option<usize> {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        let state = handle.display.lock().unwrap();
        state
            .requests
            .pending_scene
            .as_ref()
            .map(|request| match &request.action {
                scene::SceneAction::SelectBody(id) => *id,
                _ => usize::MAX,
            })
    };
    let click = |draw: &std::rc::Rc<std::cell::RefCell<ChromeDraw>>, point: egui::Pos2| {
        draw.borrow_mut()(vec![
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
        ]);
    };
    let visible = rows(&ctx);
    let root_row = visible
        .iter()
        .find(|(id, _)| *id == 1)
        .expect("root row visible");
    click(&draw, root_row.1.center());
    draw.borrow_mut()(vec![]);
    assert_eq!(queued(&desktop), Some(1), "label click did not select");
    assert_eq!(
        desktop.borrow().follow.as_ref().map(|f| f.body),
        Some(1),
        "selection changed the bound follow target"
    );
    {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        handle.display.lock().unwrap().requests.pending_scene = None;
    }

    // The arrow expands the child without queueing any selection.
    let arrows_now = arrows(&ctx);
    let arrow = arrows_now
        .iter()
        .find(|(id, _)| *id == 1)
        .expect("root arrow")
        .1;
    click(&draw, arrow.center());
    for _ in 0..4 {
        draw.borrow_mut()(vec![]);
    }
    let expanded = rows(&ctx);
    assert!(
        expanded.iter().any(|(id, _)| *id == 2),
        "arrow did not reveal the child"
    );
    // The hierarchy stays visible: the child row is indented under its
    // parent, and each arrow centers on its actual row.
    let child_row = expanded.iter().find(|(id, _)| *id == 2).unwrap().1;
    let parent_row = expanded.iter().find(|(id, _)| *id == 1).unwrap().1;
    assert!(
        child_row.left() > parent_row.left() + 8.,
        "child row is not indented under its parent: {child_row:?} vs {parent_row:?}"
    );
    // Probes accumulate across frames; the latest entry per body is the
    // settled layout of the most recent draw.
    let latest = |probes: Vec<(usize, egui::Rect)>, id: usize| {
        probes
            .iter()
            .rev()
            .find(|(probe_id, _)| *probe_id == id)
            .unwrap()
            .1
    };
    for (arrow_id, _) in arrows(&ctx).clone() {
        let arrow_rect = latest(arrows(&ctx), arrow_id);
        let row = latest(expanded.clone(), arrow_id);
        assert!(
            (arrow_rect.center().y - row.center().y).abs() < 1.,
            "arrow {arrow_id} is not centered on its row: {arrow_rect:?} vs {row:?}"
        );
    }
    {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        let state = handle.display.lock().unwrap();
        assert!(
            state.requests.pending_scene.is_none(),
            "arrow click stole the selection"
        );
    }

    // Child label selects; the bound follow identity is preserved.
    let child = expanded.iter().find(|(id, _)| *id == 2).unwrap().1;
    click(&draw, child.center());
    draw.borrow_mut()(vec![]);
    assert_eq!(queued(&desktop), Some(2));
    assert_eq!(
        desktop.borrow().follow.as_ref().map(|f| f.body),
        Some(1),
        "child selection retargeted the bound follow"
    );

    // Keyboard activation selects through the focused row label.
    {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        handle.display.lock().unwrap().requests.pending_scene = None;
    }
    let ids =
        ctx.data(|d| d.get_temp::<Vec<(usize, egui::Id)>>(egui::Id::new("outline-row-id-probes")));
    let root_id = ids
        .and_then(|ids| ids.iter().find(|(id, _)| *id == 1).map(|(_, id)| *id))
        .expect("row focus id probe");
    ctx.memory_mut(|m| m.request_focus(root_id));
    draw.borrow_mut()(vec![]);
    draw.borrow_mut()(vec![egui::Event::Key {
        key: egui::Key::Enter,
        physical_key: None,
        pressed: true,
        repeat: false,
        modifiers: Default::default(),
    }]);
    for _ in 0..2 {
        draw.borrow_mut()(vec![]);
    }
    assert_eq!(queued(&desktop), Some(1), "keyboard activation failed");

    // The World row selects its own body.
    let rows_now = rows(&ctx);
    let world_row = rows_now.iter().find(|(id, _)| *id == 0).unwrap().1;
    click(&draw, world_row.center());
    draw.borrow_mut()(vec![]);
    assert_eq!(queued(&desktop), Some(0));
}

#[test]
fn inspector_ancestor_action_uses_friendly_identity_and_movement_scope() {
    let (_ctx, desktop, draw) = chrome_harness(736., 688.);
    {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        let mut state = handle.display.lock().unwrap();
        state.document.robot = Some("robot-rover".into());
        state.robot_root = Some(1);
        state.bodies = std::sync::Arc::from(
            vec![
                crate::mujoco::NativeBody {
                    id: 0,
                    name: "world".into(),
                    parent: None,
                    mobility: crate::mujoco::BodyMobility::Fixed,
                },
                crate::mujoco::NativeBody {
                    id: 1,
                    name: "robot-rover__base_link".into(),
                    parent: Some(0),
                    mobility: crate::mujoco::BodyMobility::FreeJoint,
                },
                crate::mujoco::NativeBody {
                    id: 2,
                    name: "robot-rover__rotor_link".into(),
                    parent: Some(1),
                    mobility: crate::mujoco::BodyMobility::Fixed,
                },
            ]
            .into_boxed_slice(),
        );
        state.selection = Some(crate::mujoco::NativeSelection {
            body: 2,
            geom: None,
            point: [0.; 3],
        });
    }
    desktop.borrow_mut().inspector_open = true;
    for _ in 0..5 {
        draw.borrow_mut()(vec![]);
    }
    let texts = draw.borrow_mut()(vec![]);
    assert!(
        texts
            .iter()
            .any(|t| t.contains("Select robot body · base_link")),
        "friendly ancestor action missing: {texts:?}"
    );
    assert!(
        texts
            .iter()
            .any(|t| t.contains("Moves and forces apply to that body")),
        "movement scope missing: {texts:?}"
    );
    assert!(
        !texts
            .iter()
            .any(|t| t.contains("movable ancestor") || t.contains("robot-rover__base_link")),
        "internal identity or jargon leaked: {texts:?}"
    );
}

#[test]
fn scene_only_movable_ancestor_is_never_called_a_robot_body() {
    // A scene body below a scene-only free ancestor keeps truthful wording:
    // verified robot membership is required for robot wording.
    let (_ctx, desktop, draw) = chrome_harness(736., 688.);
    {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        let mut state = handle.display.lock().unwrap();
        state.bodies = std::sync::Arc::from(
            vec![
                crate::mujoco::NativeBody {
                    id: 0,
                    name: "world".into(),
                    parent: None,
                    mobility: crate::mujoco::BodyMobility::Fixed,
                },
                crate::mujoco::NativeBody {
                    id: 1,
                    name: "loose_box".into(),
                    parent: Some(0),
                    mobility: crate::mujoco::BodyMobility::FreeJoint,
                },
                crate::mujoco::NativeBody {
                    id: 2,
                    name: "box_handle".into(),
                    parent: Some(1),
                    mobility: crate::mujoco::BodyMobility::Fixed,
                },
            ]
            .into_boxed_slice(),
        );
        state.selection = Some(crate::mujoco::NativeSelection {
            body: 2,
            geom: None,
            point: [0.; 3],
        });
    }
    desktop.borrow_mut().inspector_open = true;
    for _ in 0..5 {
        draw.borrow_mut()(vec![]);
    }
    let texts = draw.borrow_mut()(vec![]);
    assert!(
        texts
            .iter()
            .any(|t| t.contains("Select movable body · loose_box")),
        "scene-only movable ancestor missing truthful action: {texts:?}"
    );
    assert!(
        !texts.iter().any(|t| t.contains("Select robot body")),
        "invented robot membership for a scene-only ancestor: {texts:?}"
    );
    assert!(
        texts
            .iter()
            .any(|t| t.contains("Moves and forces apply to that body")),
        "movement scope missing: {texts:?}"
    );
}

#[test]
fn help_close_stays_fully_visible_with_larger_targets() {
    for width in [320., 480., 736., 1024.] {
        let (ctx, desktop, draw) = chrome_harness(width, 560.);
        desktop.borrow_mut().help_open = true;
        desktop.borrow_mut().larger_targets = true;
        for _ in 0..4 {
            draw.borrow_mut()(vec![]);
        }
        let close = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("help-close-probe")))
            .expect("close probe");
        let modal = ctx
            .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("modal-bounds")))
            .unwrap();
        assert!(
            modal.contains_rect(close),
            "{width}: focused Close escapes the help sheet {close:?} vs {modal:?}"
        );
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width, 560.));
        assert!(
            screen.contains_rect(close),
            "{width}: Close escapes the screen {close:?}"
        );
        // Clip-aware text: the label is fully painted, not partially cut.
        let texts = draw.borrow_mut()(vec![]);
        assert!(
            texts.iter().any(|t| t.contains("Close help")),
            "{width}: Close help text is clipped: {texts:?}"
        );
    }
}

#[test]
fn admission_refusal_is_a_scoped_warning_and_never_masks_execution_state() {
    let notice = crate::notice::Notice::new(
        "Could not open scene invalid-scene.xml.",
        "The current simulation is kept. Correct the reported cause and try again.",
        "compile detail: uncompiled node of type bad",
    );
    for (running, expected) in [(true, "Running"), (false, "Paused")] {
        let (_ctx, desktop, draw) = chrome_harness(1024., 688.);
        {
            let world = desktop.borrow().world.clone();
            let handle = world.lock().unwrap();
            let mut state = handle.display.lock().unwrap();
            state.ready = true;
            state.finished = false;
            state.running = running;
            state.document_error = Some(notice.clone());
        }
        for _ in 0..4 {
            draw.borrow_mut()(vec![]);
        }
        let texts = draw.borrow_mut()(vec![]);
        assert!(
            texts.iter().any(|t| t.contains(expected)),
            "refusal masked the {expected} world: {texts:?}"
        );
        assert!(
            !texts.iter().any(|t| t == "Failed"),
            "refusal reported the retained world as Failed: {texts:?}"
        );
        assert!(
            texts
                .iter()
                .any(|t| t.contains("Could not open scene invalid-scene.xml.")),
            "scoped refusal primary missing: {texts:?}"
        );
        assert!(
            !texts
                .iter()
                .any(|t| t.contains("inspect the execution error")),
            "Open failure was called an execution error: {texts:?}"
        );
    }
    // A genuine execution failure is still Failed and keeps Error details.
    let (_ctx, desktop, draw) = chrome_harness(1024., 688.);
    {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        let mut state = handle.display.lock().unwrap();
        state.ready = true;
        state.finished = true;
        state.error = Some("Primary execution failure".into());
    }
    for _ in 0..4 {
        draw.borrow_mut()(vec![]);
    }
    let texts = draw.borrow_mut()(vec![]);
    assert!(texts.iter().any(|t| t == "Failed"));
    assert!(texts.iter().any(|t| t.contains("Error details")));
}

#[test]
fn session_ui_has_no_save_relink_or_modified_persistence_paths() {
    let (ctx, desktop, draw) = chrome_harness(1024., 688.);
    {
        let world = desktop.borrow().world.clone();
        let handle = world.lock().unwrap();
        let mut state = handle.display.lock().unwrap();
        state.ready = true;
        state.finished = true;
        state.document.scene_name = "scene.xml".into();
        state.document.robot = Some("robot-rover".into());
    }
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    // The header identity is exposed without any modification state.
    let texts = draw.borrow_mut()(vec![]);
    assert!(texts.iter().any(|t| t.contains("scene.xml")));
    assert!(
        !texts
            .iter()
            .any(|t| t.contains("Modified") || t.contains("Unmodified")),
        "dirty tracking survived: {texts:?}"
    );
    // Opening the scene menu shows the read-only session actions only.
    let menu = ctx
        .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("document-menu-probe")))
        .expect("menu probe");
    let open = |draw: &std::rc::Rc<std::cell::RefCell<ChromeDraw>>, point: egui::Pos2| {
        draw.borrow_mut()(vec![
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
        ]);
    };
    open(&draw, menu.center());
    for _ in 0..3 {
        draw.borrow_mut()(vec![]);
    }
    let texts = draw.borrow_mut()(vec![]);
    for action in ["New simulation", "Open scene…"] {
        assert!(
            texts.iter().any(|t| t.contains(action)),
            "menu lost {action}: {texts:?}"
        );
    }
    for retired in ["Save simulation", "Save as…", "Open simulation…", "Relink"] {
        assert!(
            !texts.iter().any(|t| t.contains(retired)),
            "retired persistence action {retired} survived: {texts:?}"
        );
    }
    // Departures proceed without any unsaved-changes decision.
    let mut departure = Departure::default();
    departure.begin(PendingAction::New);
    assert!(matches!(
        departure.take_ready(true, false),
        Some(PendingAction::New)
    ));
}

#[test]
#[cfg(native_qualification)]
fn invalid_scene_refusal_keeps_live_world_running_without_failure_masking() {
    // The live outcome after a scoped refusal: the retained world keeps
    // executing and the badge stays truthful while the refusal shows as a
    // scoped warning with the compile detail preserved.
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let handle = Handle::start(None).unwrap();
    let display = handle.display.clone();
    wait(&display, |s| s.ready && s.frame.is_some());
    let invalid = tempfile::tempdir().unwrap();
    let invalid_scene = invalid.path().join("invalid-scene.xml");
    std::fs::write(
        &invalid_scene,
        "<mujoco><worldbody><bad/></worldbody></mujoco>",
    )
    .unwrap();
    handle
        .edit(Edit::Scene(invalid_scene.clone()))
        .expect("stopped world admits the edit request");
    wait(&display, |s| s.document_error.is_some() && !s.document.busy);
    {
        let state = display.lock().unwrap();
        let error = state.document_error.as_ref().unwrap();
        assert!(
            error
                .primary
                .starts_with("Could not open scene invalid-scene.xml."),
            "unscoped refusal: {}",
            error.primary
        );
        assert!(
            error.action.contains("current simulation is kept"),
            "no retention wording: {}",
            error.action
        );
        assert!(
            error.details.contains("bad"),
            "compile detail lost: {}",
            error.details
        );
    }
    // The refused world still runs.
    world_send(&handle, Command::Run);
    wait(&display, |s| s.running && !s.finished);
    let world = Arc::new(Mutex::new(handle));
    let mut desktop = Desktop::new(world.clone(), display.clone());
    let draw = |desktop: &mut Desktop| {
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1024., 688.),
                )),
                ..Default::default()
            },
            |ui| desktop.draw(ui),
        );
        let texts: Vec<String> = out
            .shapes
            .iter()
            .filter_map(|shape| {
                if let egui::Shape::Text(text) = &shape.shape {
                    Some(text.galley.text().to_owned())
                } else {
                    None
                }
            })
            .collect();
        out.textures_delta.clear();
        texts
    };
    for _ in 0..6 {
        draw(&mut desktop);
        std::thread::sleep(Duration::from_millis(30));
    }
    let texts = draw(&mut desktop);
    assert!(
        texts.iter().any(|t| t == "Running"),
        "refusal masked the running world: {texts:?}"
    );
    assert!(
        !texts.iter().any(|t| t == "Failed"),
        "refusal reported the retained world as Failed: {texts:?}"
    );
    assert!(
        texts
            .iter()
            .any(|t| t.contains("Could not open scene invalid-scene.xml.")),
        "scoped refusal primary missing: {texts:?}"
    );
    world_send(&world.lock().unwrap(), Command::Stop);
    wait(&display, |s| s.finished && !s.stopping);
    drop(desktop);
    drop(world);
}

#[test]
#[cfg(native_qualification)]
fn retained_session_flows_never_write_authored_inputs() {
    // Direct evidence: a full retained session (scene selection, import,
    // placement, execution, session-only remove/undo) leaves the authored
    // scene and the complete prepared bundle byte-for-byte unchanged.
    fn digest_tree(root: &std::path::Path) -> String {
        use sha2::Digest;
        let mut hasher = sha2::Sha256::new();
        let mut paths: Vec<std::path::PathBuf> = Vec::new();
        let mut stack = vec![root.to_owned()];
        while let Some(dir) = stack.pop() {
            let mut entries: Vec<_> = std::fs::read_dir(&dir)
                .unwrap()
                .map(|e| e.unwrap().path())
                .collect();
            entries.sort();
            for path in entries {
                if path.is_dir() {
                    stack.push(path);
                } else {
                    paths.push(path);
                }
            }
        }
        paths.sort();
        for path in &paths {
            hasher.update(
                path.strip_prefix(root)
                    .unwrap()
                    .to_string_lossy()
                    .as_bytes(),
            );
            hasher.update(std::fs::read(path).unwrap());
        }
        format!("{:x}", hasher.finalize())
    }
    let build = std::path::PathBuf::from(std::env::var("PHOXAL_QUALIFICATION_BUILD").unwrap());
    let scene = std::env::var("PHOXAL_QUALIFICATION_SCENE")
        .map(std::path::PathBuf::from)
        .ok();
    let digest_input = |path: &std::path::Path| {
        if path.is_dir() {
            digest_tree(path)
        } else {
            use sha2::Digest;
            let mut hasher = sha2::Sha256::new();
            hasher.update(std::fs::read(path).unwrap());
            format!("{:x}", hasher.finalize())
        }
    };
    let before_build = digest_tree(&build);
    let before_scene = scene.as_deref().map(digest_input);
    let handle = Handle::start(None).unwrap();
    let display = handle.display.clone();
    wait(&display, |s| s.ready && s.frame.is_some());
    // Open the qualification scene explicitly (a real retained flow).
    if let Some(scene) = &scene {
        handle.edit(Edit::Scene(scene.clone())).unwrap();
        wait(&display, |s| !s.document.busy && s.document_error.is_none());
    }
    handle
        .edit(Edit::Import {
            build: build.clone(),
            target: "robot_mount".into(),
            position: [1., 0., 0.],
            quaternion: [1., 0., 0., 0.],
        })
        .unwrap();
    wait(&display, |s| {
        !s.document.busy && s.document.candidate.is_some()
    });
    handle.edit(Edit::Place).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_some());
    world_send(&handle, Command::Run);
    wait(&display, |s| s.running && !s.finished);
    world_send(&handle, Command::Pause);
    wait(&display, |s| !s.running && !s.finished);
    world_send(&handle, Command::Stop);
    wait(&display, |s| s.finished && !s.stopping);
    handle.edit(Edit::Remove).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_none());
    handle.edit(Edit::Undo).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_some());
    world_send(&handle, Command::Run);
    wait(&display, |s| s.running && !s.finished);
    world_send(&handle, Command::Stop);
    wait(&display, |s| s.finished && !s.stopping);
    drop(handle);
    assert_eq!(digest_tree(&build), before_build, "prepared bundle changed");
    if let (Some(scene), Some(before)) = (&scene, before_scene) {
        assert_eq!(digest_input(scene), before, "authored scene changed");
    }
}

#[test]
fn inspector_numeric_paint_clears_active_scrollbar_across_production_layouts() {
    let xml = r#"<mujoco><worldbody><geom type="plane" size="10 10 0.1"/><body name="robot-rover__base_link" pos="1 0 0.21"><freejoint/><geom type="sphere" size="0.2"/></body></worldbody></mujoco>"#;
    let model = crate::mujoco::Model::from_xml(xml.as_bytes()).unwrap();
    let workspace = crate::mujoco::Workspace::new(&model).unwrap();
    let snapshot = workspace.snapshot().unwrap();
    let camera = workspace.default_view_camera();
    for scale in [1.0f32, 1.4] {
        for larger in [false, true] {
            for width in [736., 320., 1024., 480.] {
                for motion in [false, true] {
                    let (ctx, desktop, draw) = chrome_harness(width, 560.);
                    if scale != 1.0 {
                        ctx.all_styles_mut(|style| {
                            let body = style.text_styles.get(&egui::TextStyle::Body).cloned();
                            if let Some(font) = body {
                                style.text_styles.insert(
                                    egui::TextStyle::Body,
                                    egui::FontId::proportional(font.size * scale),
                                );
                            }
                        });
                    }
                    {
                        let world = desktop.borrow().world.clone();
                        let world = world.lock().unwrap();
                        let mut state = world.display.lock().unwrap();
                        state.document.robot = Some("robot-rover".into());
                        state.robot_root = Some(1);
                        state.bodies = std::sync::Arc::from(
                            vec![
                                crate::mujoco::NativeBody {
                                    id: 0,
                                    name: "world".into(),
                                    parent: None,
                                    mobility: crate::mujoco::BodyMobility::Fixed,
                                },
                                crate::mujoco::NativeBody {
                                    id: 1,
                                    name: "robot-rover__base_link".into(),
                                    parent: Some(0),
                                    mobility: crate::mujoco::BodyMobility::FreeJoint,
                                },
                            ]
                            .into_boxed_slice(),
                        );
                        state.selection = Some(crate::mujoco::NativeSelection {
                            body: 1,
                            geom: None,
                            point: [0.; 3],
                        });
                        state.requests.presented =
                            Some(std::sync::Arc::new(crate::app::scene::PresentedView {
                                identity: crate::app::scene::FrameIdentity {
                                    epoch: crate::app::scene::SceneEpoch {
                                        execution: "chrome".into(),
                                        model: model.identity(),
                                        generation: 0,
                                    },
                                    serial: 1,
                                },
                                camera,
                                resolution: [4, 3],
                                snapshot: snapshot.clone(),
                            }));
                    }
                    {
                        let mut desktop = desktop.borrow_mut();
                        desktop.inspector_open = true;
                        desktop.motion_tab = motion;
                        desktop.larger_targets = larger;
                    }
                    for _ in 0..5 {
                        draw.borrow_mut()(vec![]);
                    }

                    let mut seen = std::collections::BTreeSet::new();
                    let mut observed_bar = false;
                    for step in 0..70 {
                        let point = if width < 600. {
                            egui::pos2(
                                width * 0.5,
                                desktop.borrow().last_viewport.unwrap().bottom() + 35.,
                            )
                        } else {
                            egui::pos2(width - 120., 320.)
                        };
                        draw.borrow_mut()(vec![
                            egui::Event::PointerMoved(point),
                            egui::Event::MouseWheel {
                                unit: egui::MouseWheelUnit::Point,
                                phase: egui::TouchPhase::Move,
                                delta: egui::vec2(0., if step < 35 { -12. } else { 12. }),
                                modifiers: Default::default(),
                            },
                        ]);
                        for _ in 0..3 {
                            draw.borrow_mut()(vec![egui::Event::PointerMoved(point)]);
                        }
                        let shapes = ctx
                            .data(|d| d.get_temp::<Vec<egui::Shape>>(egui::Id::new("shape-probes")))
                            .unwrap();
                        // Actual filled track/thumb shapes emitted by the scroll owner.
                        // The Inspector is the only scroller in this fixture.
                        let bars: Vec<_> = shapes
                            .iter()
                            .filter_map(|s| match s {
                                egui::Shape::Rect(r)
                                    if r.rect.width() <= 10.1
                                        && r.rect.width() > 0.
                                        && r.rect.height() > 20.
                                        && r.fill.a() > 0 =>
                                {
                                    Some(r.rect)
                                }
                                _ => None,
                            })
                            .collect();
                        observed_bar |= !bars.is_empty();
                        let texts = ctx
                            .data(|d| {
                                d.get_temp::<Vec<(String, egui::Rect)>>(egui::Id::new(
                                    "text-probes",
                                ))
                            })
                            .unwrap();
                        for (text, rect) in texts {
                            if text.parse::<f64>().is_ok() && text.contains('.') {
                                seen.insert(text.clone());
                                for bar in &bars {
                                    assert!(
                                        rect.right() + 3.
                                            <= bar.right()
                                                - ctx
                                                    .style_of(egui::Theme::Dark)
                                                    .spacing
                                                    .scroll
                                                    .bar_width,
                                        "{width} x{scale} larger={larger} motion={motion}: numeric {text} {rect:?} overlaps active scrollbar/gap {bar:?}"
                                    );
                                }
                            }
                        }
                    }
                    assert!(observed_bar, "{width}: no actual active scrollbar painted");
                    for value in ["1.000", "0.000", "0.210"] {
                        assert!(
                            seen.contains(value),
                            "{width} x{scale} larger={larger} motion={motion}: full {value} never scroll-reachable: {seen:?}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn outline_heading_and_scene_only_robot_scope_use_both_live_layouts() {
    for width in [1024., 736., 480., 320.] {
        for selected in [None, Some(0), Some(1), Some(2)] {
            let (_, desktop, draw) = chrome_harness(width, 821.);
            {
                let mut desktop = desktop.borrow_mut();
                desktop.outline_open = true;
                desktop.inspector_open = true;
                let world = desktop.world.lock().unwrap();
                let mut state = world.display.lock().unwrap();
                state.bodies = Arc::from(vec![
                    crate::mujoco::NativeBody {
                        id: 0,
                        name: "world".into(),
                        parent: None,
                        mobility: crate::mujoco::BodyMobility::Fixed,
                    },
                    crate::mujoco::NativeBody {
                        id: 1,
                        name: "loose_box".into(),
                        parent: Some(0),
                        mobility: crate::mujoco::BodyMobility::FreeJoint,
                    },
                    crate::mujoco::NativeBody {
                        id: 2,
                        name: "box_handle".into(),
                        parent: Some(1),
                        mobility: crate::mujoco::BodyMobility::Fixed,
                    },
                ]);
                state.selection = selected.map(|body| crate::mujoco::NativeSelection {
                    body,
                    geom: None,
                    point: [0.; 3],
                });
            }
            for _ in 0..5 {
                draw.borrow_mut()(vec![]);
            }
            let texts = draw.borrow_mut()(vec![]);
            assert!(
                texts.iter().any(|t| t == "Outline"),
                "{width}: heading {texts:?}"
            );
            assert!(
                !texts.iter().any(|t| t == "Scene" || t == "Robot in scene"),
                "{width}: orphan/misnamed group {texts:?}"
            );
        }
    }
}

#[test]
fn genuine_disclosure_painted_mesh_points_right_collapsed_and_down_expanded() {
    for collapsed in [false, true] {
        let ctx = egui::Context::default();
        theme::apply(&ctx);
        let mut meshes = Vec::new();
        for _ in 0..5 {
            let mut output = ctx.run_ui(Default::default(), |ui| {
                icons::Icon::ChevronDown.paint_at_rotated(
                    ui,
                    egui::Rect::from_min_size(egui::pos2(10., 10.), egui::vec2(20., 20.)),
                    theme::TEXT,
                    16.,
                    collapsed,
                );
            });
            output.textures_delta.clear();
            meshes = output
                .shapes
                .into_iter()
                .filter_map(|s| match s.shape {
                    egui::Shape::Mesh(m) => Some(m),
                    _ => None,
                })
                .collect();
        }
        let mesh = meshes
            .last()
            .expect("genuine SVG texture must actually paint");
        // Transform the authentic SVG's tip and two arm ends through the
        // emitted texture mesh. A rotation flag alone cannot prove direction.
        assert!(include_str!("../../resources/icons/chevron-down.svg").contains("m6 9 6 6 6-6"));
        let map = |uv: egui::Pos2| {
            let a = mesh
                .vertices
                .iter()
                .find(|v| v.uv == egui::pos2(0., 0.))
                .unwrap()
                .pos;
            let b = mesh
                .vertices
                .iter()
                .find(|v| v.uv == egui::pos2(1., 0.))
                .unwrap()
                .pos;
            let c = mesh
                .vertices
                .iter()
                .find(|v| v.uv == egui::pos2(0., 1.))
                .unwrap()
                .pos;
            a + (b - a) * uv.x + (c - a) * uv.y
        };
        let tip = map(egui::pos2(0.5, 15. / 24.));
        let arms = (map(egui::pos2(6. / 24., 9. / 24.)).to_vec2()
            + map(egui::pos2(18. / 24., 9. / 24.)).to_vec2())
            * 0.5;
        let direction = tip.to_vec2() - arms;
        if collapsed {
            assert!(
                direction.x > 3. && direction.y.abs() < 0.1,
                "collapsed glyph points {direction:?}"
            );
        } else {
            assert!(
                direction.y > 3. && direction.x.abs() < 0.1,
                "expanded glyph points {direction:?}"
            );
        }
    }
}

#[test]
fn robot_scope_keeps_imported_actions_and_conditional_undo_for_every_selection() {
    for selected in [None, Some(0), Some(1), Some(2)] {
        for (robot, undo) in [(false, false), (true, false), (false, true)] {
            let (ctx, desktop, _) = chrome_harness(736., 560.);
            let mut state = RunState {
                snapshot: crate::app::RunSnapshot {
                    selection: selected.map(|body| crate::mujoco::NativeSelection {
                        body,
                        geom: None,
                        point: [0.; 3],
                    }),
                    ..Default::default()
                },
                ..Default::default()
            };
            state.document.robot = robot.then(|| "robot-rover".into());
            state.document.removed_robot = undo.then(|| "robot-rover".into());
            let mut output = ctx.run_ui(Default::default(), |ui| {
                desktop.borrow_mut().robot_actions(ui, &mut state, true)
            });
            output.textures_delta.clear();
            let texts: Vec<_> = output
                .shapes
                .iter()
                .filter_map(|shape| match &shape.shape {
                    egui::Shape::Text(t) => Some(t.galley.text()),
                    _ => None,
                })
                .collect();
            assert_eq!(texts.contains(&"Robot in scene"), robot);
            assert_eq!(texts.contains(&"Remove from scene"), robot);
            assert_eq!(texts.contains(&"Undo removal"), undo);
            assert_eq!(texts.contains(&"Removed robot"), undo);
            assert_eq!(texts.contains(&"robot-rover"), robot || undo);
            assert_eq!(
                texts.contains(&"Restores robot-rover at its original placement"),
                undo
            );
            assert!(!texts.contains(&"Robot removed from scene"));
            if !robot && !undo {
                assert!(
                    output.shapes.is_empty(),
                    "orphan separator/heading: {texts:?}"
                );
            }
        }
    }
}

#[test]
#[cfg(native_qualification)]
fn removed_robot_group_uses_retained_owner_identity_and_original_placement() {
    let handle = Handle::start(None).unwrap();
    let display = handle.display.clone();
    wait(&display, |s| s.ready && s.finished);
    handle
        .edit(Edit::Import {
            build: std::env::var_os("PHOXAL_QUALIFICATION_BUILD")
                .unwrap()
                .into(),
            target: "robot_mount".into(),
            position: [1., 0., 0.],
            quaternion: [1., 0., 0., 0.],
        })
        .unwrap();
    wait(&display, |s| {
        !s.document.busy && s.document.candidate.is_some()
    });
    handle.edit(Edit::Place).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_some());
    let original = display.lock().unwrap().native_state.clone().unwrap();
    assert_eq!(original.qpos()[0], 1.);
    world_send(&handle, Command::Run);
    wait(&display, |s| s.running && !s.finished);
    world_send(&handle, Command::Stop);
    wait(&display, |s| s.finished && !s.stopping);
    handle.edit(Edit::Remove).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_none());
    let world = Arc::new(Mutex::new(handle));
    for scale in [1.0, 1.4] {
        for larger in [false, true] {
            for width in [1024., 736., 480., 320.] {
                let ctx = egui::Context::default();
                theme::apply(&ctx);
                ctx.all_styles_mut(|style| {
                    for font in style.text_styles.values_mut() {
                        font.size *= scale;
                    }
                });
                let mut desktop = Desktop::new(world.clone(), display.clone());
                desktop.inspector_open = true;
                desktop.larger_targets = larger;
                let draw = |desktop: &mut Desktop, events| {
                    let mut output = ctx.run_ui(
                        egui::RawInput {
                            screen_rect: Some(egui::Rect::from_min_size(
                                egui::Pos2::ZERO,
                                egui::vec2(width, 688.),
                            )),
                            events,
                            ..Default::default()
                        },
                        |ui| desktop.draw(ui),
                    );
                    output.textures_delta.clear();
                    output
                        .shapes
                        .into_iter()
                        .filter_map(|shape| {
                            if let egui::Shape::Text(text) = &shape.shape {
                                shape
                                    .clip_rect
                                    .expand(0.5)
                                    .contains_rect(shape.shape.visual_bounding_rect())
                                    .then(|| text.galley.text().to_owned())
                            } else {
                                None
                            }
                        })
                        .collect::<Vec<_>>()
                };
                for _ in 0..6 {
                    draw(&mut desktop, vec![]);
                }
                let mut seen = std::collections::BTreeSet::new();
                let mut reachable = false;
                for step in 0..45 {
                    let texts = draw(&mut desktop, vec![]);
                    eprintln!(
                        "REMOVED_PANEL width={width} scale={scale} larger={larger} step={step} visible={texts:?}"
                    );
                    assert!(
                        !texts
                            .iter()
                            .any(|s| s == "Robot in scene" || s == "Robot removed from scene")
                    );
                    seen.extend(texts);
                    let probe = ctx.data(|d| {
                        d.get_temp::<(egui::Rect, egui::Id, bool)>(egui::Id::new(
                            "undo-action-id-probe",
                        ))
                    });
                    if let Some((rect, _, enabled)) = probe {
                        reachable |=
                            enabled && seen.contains("Undo removal") && rect.bottom() < 646.;
                    }
                    let point = if width < 600. {
                        egui::pos2(width * 0.5, 560.)
                    } else {
                        egui::pos2(width - 120., 320.)
                    };
                    draw(
                        &mut desktop,
                        vec![
                            egui::Event::PointerMoved(point),
                            egui::Event::MouseWheel {
                                unit: egui::MouseWheelUnit::Point,
                                phase: egui::TouchPhase::Move,
                                delta: egui::vec2(0., -12.),
                                modifiers: Default::default(),
                            },
                        ],
                    );
                }
                for text in [
                    "Removed robot",
                    "robot-rover",
                    "Undo removal",
                    "Restores robot-rover at its original placement",
                ] {
                    assert!(
                        seen.contains(text),
                        "{width} scale={scale} larger={larger}: full text not clip-aware reachable: {text}; {seen:?}"
                    );
                }
                assert!(reachable, "Undo not reachable at {width}");
                {
                    let (_, id, enabled) = ctx
                        .data(|d| {
                            d.get_temp::<(egui::Rect, egui::Id, bool)>(egui::Id::new(
                                "undo-action-id-probe",
                            ))
                        })
                        .unwrap();
                    assert!(enabled);
                    ctx.memory_mut(|m| m.request_focus(id));
                    draw(
                        &mut desktop,
                        vec![egui::Event::Key {
                            key: egui::Key::Enter,
                            physical_key: None,
                            pressed: true,
                            repeat: false,
                            modifiers: Default::default(),
                        }],
                    );
                    wait(&display, |s| !s.document.busy && s.document.robot.is_some());
                    assert_eq!(
                        display.lock().unwrap().document.robot.as_deref(),
                        Some("robot-rover")
                    );
                    assert_eq!(
                        display
                            .lock()
                            .unwrap()
                            .native_state
                            .as_ref()
                            .unwrap()
                            .qpos(),
                        original.qpos()
                    );
                    let mut restored = std::collections::BTreeSet::new();
                    for _ in 0..35 {
                        let visible = draw(&mut desktop, vec![]);
                        assert!(!visible.iter().any(|s| s == "Removed robot"));
                        restored.extend(visible);
                        let point = if width < 600. {
                            egui::pos2(width * 0.5, 560.)
                        } else {
                            egui::pos2(width - 120., 320.)
                        };
                        draw(
                            &mut desktop,
                            vec![
                                egui::Event::PointerMoved(point),
                                egui::Event::MouseWheel {
                                    unit: egui::MouseWheelUnit::Point,
                                    phase: egui::TouchPhase::Move,
                                    delta: egui::vec2(0., 12.),
                                    modifiers: Default::default(),
                                },
                            ],
                        );
                    }
                    assert!(
                        restored.contains("Robot in scene"),
                        "restored heading not reachable {width}: {restored:?}"
                    );
                    assert!(restored.contains("robot-rover"));
                    world.lock().unwrap().edit(Edit::Remove).unwrap();
                    wait(&display, |s| !s.document.busy && s.document.robot.is_none());
                }
            }
        }
    }
    world.lock().unwrap().shutdown().unwrap();
}

#[test]
fn removed_robot_action_preserves_authoritative_restriction_priority() {
    for (cleanup, stopping, busy, stopped, expected) in [
        (
            true,
            false,
            true,
            false,
            "Cleanup incomplete. Resolve the shutdown failure and confirm cleanup before undoing",
        ),
        (
            false,
            true,
            true,
            false,
            "Cleanup incomplete. Resolve the shutdown failure and confirm cleanup before undoing",
        ),
        (false, false, true, false, "Updating simulation…"),
        (
            false,
            false,
            false,
            false,
            "Stop the simulation before undoing the removal",
        ),
        (
            false,
            false,
            false,
            true,
            "Restores robot-rover at its original placement",
        ),
    ] {
        let (ctx, desktop, _) = chrome_harness(736., 560.);
        let mut state = RunState {
            snapshot: crate::app::RunSnapshot {
                cleanup_failed: cleanup,
                stopping,
                ..Default::default()
            },
            ..Default::default()
        };
        state.document.removed_robot = Some("robot-rover".into());
        state.document.busy = busy;
        let mut output = ctx.run_ui(Default::default(), |ui| {
            desktop.borrow_mut().robot_actions(ui, &mut state, stopped)
        });
        output.textures_delta.clear();
        let texts: Vec<_> = output
            .shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::Shape::Text(text) => Some(text.galley.text()),
                _ => None,
            })
            .collect();
        assert!(texts.contains(&expected), "restriction missing: {texts:?}");
        assert!(texts.contains(&"Removed robot") && texts.contains(&"robot-rover"));
        let (_, _, enabled) = ctx
            .data(|d| {
                d.get_temp::<(egui::Rect, egui::Id, bool)>(egui::Id::new("undo-action-id-probe"))
            })
            .unwrap();
        assert_eq!(enabled, !cleanup && !stopping && !busy && stopped);
    }
}

#[test]
fn application_signal_close_logic_waits_for_owner_without_paint_and_keeps_restart_fence() {
    let (ctx, desktop, _) = chrome_harness(736., 560.);
    let display = desktop.borrow().display.clone();
    {
        let mut state = display.lock().unwrap();
        state.close_requested = true;
        state.finished = false;
        state.document.busy = true;
        state.cleanup_failed = true;
        state.error = Some(Notice::new(
            "Original failure",
            "Resolve cleanup",
            "Unconfirmed",
        ));
    }
    let input = egui::RawInput::default();
    let deferred = ctx.run_logic(&input, |ctx| {
        desktop.borrow_mut().application_termination(ctx)
    });
    assert!(!desktop.borrow().allow_close);
    assert!(
        !deferred
            .viewport_commands
            .get(&egui::ViewportId::ROOT)
            .is_some_and(|cmds| cmds.contains(&egui::ViewportCommand::Close))
    );
    {
        let mut state = display.lock().unwrap();
        state.finished = true;
        state.document.busy = false;
    }
    let ended = ctx.run_logic(&input, |ctx| {
        desktop.borrow_mut().application_termination(ctx)
    });
    assert!(desktop.borrow().allow_close);
    assert!(
        ended.viewport_commands[&egui::ViewportId::ROOT].contains(&egui::ViewportCommand::Close)
    );
    let state = display.lock().unwrap();
    assert!(state.cleanup_failed);
    assert_eq!(state.error.as_ref().unwrap().primary, "Original failure");
    drop(state);
    assert!(
        desktop
            .borrow()
            .world
            .lock()
            .unwrap()
            .send(Command::Run)
            .is_err()
    );
    assert!(
        desktop
            .borrow()
            .world
            .lock()
            .unwrap()
            .edit(Edit::New)
            .is_err()
    );
}

#[test]
#[cfg(native_qualification)]
fn production_root_close_after_robot_run_and_new_reissues_then_accepts_and_joins() {
    let handle = Handle::start(None).unwrap();
    let display = handle.display.clone();
    wait(&display, |s| s.ready && s.finished);
    handle
        .edit(Edit::Import {
            build: std::env::var_os("PHOXAL_QUALIFICATION_BUILD")
                .unwrap()
                .into(),
            target: "robot_mount".into(),
            position: [1., 0., 0.],
            quaternion: [1., 0., 0., 0.],
        })
        .unwrap();
    wait(&display, |s| {
        !s.document.busy && s.document.candidate.is_some()
    });
    handle.edit(Edit::Place).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_some());
    world_send(&handle, Command::Run);
    wait(&display, |s| s.running && !s.finished);
    world_send(&handle, Command::Pause);
    wait(&display, |s| !s.running && !s.finished);
    world_send(&handle, Command::Stop);
    wait(&display, |s| s.finished && !s.stopping);
    handle.edit(Edit::Remove).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_none());
    handle.edit(Edit::Undo).unwrap();
    wait(&display, |s| !s.document.busy && s.document.robot.is_some());
    handle.edit(Edit::New).unwrap();
    wait(&display, |s| {
        !s.document.busy && s.document.robot.is_none() && s.ready && s.finished
    });
    let world = Arc::new(Mutex::new(handle));
    let mut desktop = Desktop::new(world.clone(), display);
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    for pass in 0..2 {
        let mut input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(736., 688.),
            )),
            ..Default::default()
        };
        input
            .viewports
            .get_mut(&egui::ViewportId::ROOT)
            .unwrap()
            .events
            .push(egui::ViewportEvent::Close);
        let mut output = ctx.run_ui(input, |ui| desktop.draw(ui));
        output.textures_delta.clear();
        let commands = &output.viewport_output[&egui::ViewportId::ROOT].commands;
        eprintln!(
            "ROOT_CLOSE_PRODUCTION pass={pass} allow_close={} commands={commands:?}",
            desktop.allow_close
        );
        assert!(desktop.allow_close && desktop.departure.pending.is_none());
        // eframe 0.36.2 cancels the root close if CancelClose occurs anywhere
        // in the first output; egui-winit Close queues the next Close event.
        // That next root event must be accepted, not cancelled again.
        assert_eq!(
            commands.contains(&egui::ViewportCommand::CancelClose),
            pass == 0
        );
        if pass == 0 {
            assert!(commands.contains(&egui::ViewportCommand::Close));
        }
    }
    assert!(world.lock().unwrap().shutdown().is_ok());
}

#[test]
fn short_narrow_context_paints_reserved_surface_without_reallocating_chrome() {
    for larger in [false, true] {
        for inspector in [true, false] {
            let (ctx, desktop, draw) = chrome_harness(320., 688.);
            {
                let mut app = desktop.borrow_mut();
                app.larger_targets = larger;
                app.inspector_open = inspector;
                app.outline_open = !inspector;
                app.display.lock().unwrap().document = Default::default();
            }
            for _ in 0..4 {
                draw.borrow_mut()(vec![]);
            }
            let strip = ctx
                .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("context-strip-probe")))
                .unwrap();
            let footer = ctx
                .data(|d| d.get_temp::<egui::Rect>(egui::Id::new("footer-probe")))
                .unwrap();
            let app = desktop.borrow();
            let canvas = app.last_viewport.unwrap();
            let toolbars = app.captured[..2].to_vec();
            println!(
                "SURFACE larger={larger} inspector={inspector} strip={strip:?} canvas={canvas:?} footer={footer:?} toolbars={toolbars:?}"
            );
            assert!((strip.height() - (688. - canvas.top()) * 0.2).abs() < 0.1);
            assert!((strip.width() - canvas.width()).abs() < 0.1);
            assert!(strip.top() >= canvas.bottom() && footer.top() >= strip.bottom() - 0.5);
            assert!(toolbars.iter().all(|r| canvas.contains_rect(*r)));
            let shapes = ctx
                .data(|d| {
                    d.get_temp::<Vec<egui::epaint::ClippedShape>>(egui::Id::new(
                        "clipped-shape-probes",
                    ))
                })
                .unwrap();
            for y in 0..strip.height().ceil() as usize {
                for x in [1., strip.width() * 0.5, strip.width() - 1.] {
                    let point =
                        strip.min + egui::vec2(x, (y as f32 + 0.5).min(strip.height() - 0.1));
                    assert!(
                        shapes.iter().any(|s| match &s.shape {
                            egui::Shape::Rect(r) =>
                                r.fill == theme::SURFACE
                                    && r.rect.contains(point)
                                    && s.clip_rect.contains(point),
                            _ => false,
                        }),
                        "unpainted reserved surface at {point:?}, larger={larger}, inspector={inspector}"
                    );
                }
            }
        }
    }
}

#[test]
#[cfg(native_qualification)]
fn startup_scene_identity_reaches_production_header_for_default_and_custom_cli() {
    use clap::Parser as _;
    for custom in [false, true] {
        let options = custom.then(|| {
            let build = std::env::var("PHOXAL_QUALIFICATION_BUILD").unwrap();
            let scene = std::env::var("PHOXAL_QUALIFICATION_SCENE").unwrap();
            let cli = crate::config::Cli::try_parse_from([
                "phoxal-simulator",
                "run",
                &scene,
                "--build",
                &build,
                "--paused",
            ])
            .unwrap();
            let Some(crate::config::AppCommand::Run(args)) = cli.command else {
                panic!("run command")
            };
            args.into_options()
        });
        let handle = Handle::start(options).unwrap();
        let display = handle.display.clone();
        let owner = Arc::new(Mutex::new(handle));
        let observed = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let deadline = Instant::now() + Duration::from_secs(30);
            loop {
                let s = display.lock().unwrap();
                if s.ready && s.finished != custom {
                    break;
                }
                assert!(Instant::now() < deadline, "startup {:?}", s.error);
                drop(s);
                std::thread::sleep(Duration::from_millis(5));
            }
            let ctx = egui::Context::default();
            theme::apply(&ctx);
            let mut app = Desktop::new(owner.clone(), display.clone());
            let mut visible = Vec::new();
            for _ in 0..3 {
                let mut output = ctx.run_ui(
                    egui::RawInput {
                        screen_rect: Some(egui::Rect::from_min_size(
                            egui::Pos2::ZERO,
                            egui::vec2(1024., 736.),
                        )),
                        ..Default::default()
                    },
                    |ui| app.draw(ui),
                );
                output.textures_delta.clear();
                visible = output
                    .shapes
                    .iter()
                    .filter_map(|s| match &s.shape {
                        egui::Shape::Text(t)
                            if s.clip_rect
                                .expand(0.5)
                                .contains_rect(s.shape.visual_bounding_rect()) =>
                        {
                            Some(t.galley.text().to_owned())
                        }
                        _ => None,
                    })
                    .collect();
            }
            let expected = if custom { "scene.xml" } else { "Default scene" };
            println!(
                "STARTUP_HEADER custom={custom} published_name={:?} visible={visible:?}",
                display.lock().unwrap().document.scene_name
            );
            assert!(
                visible.iter().any(|t| t == expected),
                "actual reachable header must name {expected}: {visible:?}"
            );
        }));
        owner.lock().unwrap().shutdown().unwrap();
        if let Err(panic) = observed {
            std::panic::resume_unwind(panic);
        }
    }
}
