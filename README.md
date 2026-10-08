# Phoxal Simulator

Native MuJoCo simulation and desktop controls for prepared Phoxal robots on Linux and macOS.
The simulator owns physics, rendering, native runtime setup and execution cleanup.
Robot source preparation belongs to cargo-phoxal.

## Install and set up

```sh
cargo install phoxal-simulator --locked
cargo install cargo-phoxal --version 0.4.1 --locked
phoxal-simulator setup
```

Setup explicitly downloads the checksum-pinned official MuJoCo 3.12.0 prebuilt runtime, verifies extraction and native API admission, and installs it with licenses/provenance under `~/.phoxal/simulator`.
Supported prebuilt targets are macOS arm64/x86_64 and GNU Linux aarch64/x86_64.
Setup is cancellable, serialized and atomic; a verified install can be reused offline.
Setup reports phases and measured download sizes on stderr, with a five-second per-phase heartbeat while waiting.
Supported interactive terminals show one compact live line, with measured transfer progress when the download total is known.
Redirected output, CI, dumb and narrow terminals use plain lines; resizing switches safely to plain feedback.
The final readiness result stays on stdout.
Ordinary starts never download a runtime, and there is no graphical installer.

```sh
phoxal-simulator
```

The idle window opens without a native library or prepared robot.
A background check reports availability and a specific recovery action without starting participants.
Open simulation selects a runnable build directory and scene.
Native execution needs a working graphical session for desktop rendering; headless execution does not.

For a separate installation directory, use the same explicit root on setup and subsequent commands:

```sh
phoxal-simulator --runtime-root '/chosen/runtime directory' setup
phoxal-simulator --runtime-root '/chosen/runtime directory'
```

An existing exact 3.12.0 library can be selected with `PHOXAL_MUJOCO_LIBRARY=/path/to/library`.
This strict override takes precedence; repair or remove it if it fails rather than expecting setup to replace it.
Without an override, discovery tries managed setup and bounded platform candidates, continuing past incompatible candidates.
Only successful complete native admission is cached, so repair can be retried in the same process.

## Run a robot

From the robot repository:

```sh
cargo phoxal simulation simulation/scene.xml
```

Cargo-phoxal prepares sources and the executable graph; simulator consumes that runnable build.
For an already prepared build:

```sh
phoxal-simulator run /path/to/scene.xml --build /path/to/build
```

Use `--paused` for paused startup or `--headless --duration 10s` for finite headless execution.
Scenario runs retain every boundary for evidence; ordinary desktop viewing retains bounded current state.
Hardware actuation is outside this simulation workflow.

## Controls

- **Run / Pause:** resume or freeze logical time, native world, runtime state, admitted authority and logical leases.
- **Step:** one normal native boundary while paused.
- **Reset:** restore the original scene and reset the execution while paused.
- **Cancel startup / Stop:** cancel acquisition or release authority and shut down owned execution.
- **Restart:** fresh execution after confirmed cleanup.
- **Realtime / Fast:** paced or uncapped boundary execution, without changing quantum, cadence or leases.

Sim is logical time; Wall excludes paused idle/setup; Speed is the recent achieved rate and shows `--` while paused or before enough active samples.
Slow hosts report their actual rate rather than skipping work.
Closing the window cancels and joins owned work.
Unsuccessful or unconfirmed authority/session/process cleanup disables restart and retains the primary cause and cleanup details.

Click a native body or select it in Scene; a miss clears selection.
Right drag orbits, Shift-right/middle drag pans, scroll zooms, Shift-scroll pans, and backend pinch zooms.
Focus selected frames the selected body; Default view restores the model camera.
Scene names/IDs belong to the native model, not inferred service identities.
For an attached child, Select movable ancestor explicitly selects the nearest ancestor with its own free joint.

Primary drag applies bounded physical spring/damping force while running, without teleporting qpos.
Paused translation is allowed only on an eligible body's own free joint, preserves orientation, resets its six velocity DOFs and forwards authoritative state without advancing time.
Fixed/constrained bodies refuse pose editing.
Release, Escape, focus loss, pause, reset, stop and execution replacement clear simulator-owned drag force without clearing unrelated forces.
A stationary held gesture renews a 250 ms wall-clock UI deadline, independent of robot logical leases.
Reset restores the authored pose.

Robot body evidence refers to the physical root-body origin, not footprint or center of mass.
Position and both velocity vectors use world axes; orientation is body-to-world unit quaternion wxyz and linear velocity is at that origin.

## Troubleshooting

The visible notice shows what failed, why and the permitted next action; details retain exact phase/path, child stderr and cleanup outcome.
Missing/incompatible runtime: run the displayed contextual setup command or repair/remove the strict override, then retry.
Invalid model/scene: repair the selected model/resources; installing MuJoCo does not repair authored geometry.
Graphics failure: use a working graphical session or the headless command.
Unconfirmed shutdown: confirm remote authority/session and owned process cleanup, then close and reopen; same-process restart remains fenced.
Actual desktop and held-gesture acceptance still requires a usable graphical host; offscreen renders and injected commands do not prove it.

## Development

```sh
cargo build --locked
cargo test --locked --test runtime_loading
cargo fmt --check
cargo clippy --locked --all-targets -- -D warnings
```

Build/help/version/stage/setup need no preinstalled engine.
Native tests additionally require the exact admitted library; explicit external-worker tests need their documented build/scene prerequisites.
CI selects engine-free tests on Linux and macOS and validates strict lint/docs.
The private binding adapts upstream mujoco-rs 6.0.1+mj-3.12.0 with retained licenses and generated runtime dispatch; generated ABI files are not hand-edited.
Releases use ordinary crates.io publication through release-plz.

## License

AGPL-3.0-only. See [LICENSE](LICENSE).
