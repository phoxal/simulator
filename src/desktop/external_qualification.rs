use super::tests::wait;
use super::*;
use std::time::{Duration, Instant};

/// This driver knows native controls and commands, not any participant implementation.
#[test]
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
            let display = Arc::new(Mutex::new(RunState::default()));
            let mut owner = Handle::qualification(options, display.clone()).unwrap();
            wait(&display, |state| state.ready);
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let advance = |target: u64| {
                    while display.lock().unwrap().boundary < target {
                        let next = display.lock().unwrap().boundary + 1;
                        owner.send(Command::Step).unwrap();
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
                owner.send(Command::Run).unwrap();
                wait(&display, |state| state.running && state.boundary >= 30);
                owner.send(Command::Pause).unwrap();
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
                owner.send(Command::Step).unwrap();
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
                owner.send(Command::Run).unwrap();
                wait(&display, |state| {
                    state.running && state.boundary >= boundary + 7
                });
                owner.send(Command::Pause).unwrap();
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
            let _ = owner.send(Command::Stop);
            assert!(owner.shutdown().is_ok());
            assert!(display.lock().unwrap().finished);
            println!("EXTERNAL_CLEANUP phase={phase} case={case_index} worker joined");
            if let Err(payload) = outcome {
                std::panic::resume_unwind(payload);
            }
        }
    }
}
