# Phoxal Simulator

`phoxal-simulator` owns native simulation, the desktop controls, supervisor launch and cleanup, and simulation evidence.
Robot source preparation belongs to the separate `cargo-phoxal` command.
Linux and macOS are supported.
Windows is unsupported.

## Install

```sh
cargo install phoxal-simulator
cargo install cargo-phoxal
phoxal-simulator --help
```

Installation, help, version, and scene-resource staging do not require MuJoCo.
The executable dynamically loads MuJoCo only for native operations.
No native library is downloaded, installed, or bundled.

Native simulation requires a user-managed MuJoCo **3.12.0** shared library and a working graphics environment for desktop rendering.
The simulator checks the native version before accessing engine layouts or launching the supervisor.
Unsupported versions and missing libraries produce simulator-owned errors.
Set an explicit library file when it is outside the supported system locations:

```sh
export PHOXAL_MUJOCO_LIBRARY=/path/to/libmujoco.3.12.0.dylib
```

Linux libraries normally use `libmujoco.so.3.12.0`.
Discovery also tries the platform library search path, `/usr/local/lib`, the macOS Homebrew `/opt/homebrew/lib` directory, the system MuJoCo framework, and the framework inside `/Applications/MuJoCo.app` or `/Applications/MuJoCoStudio.app`.
For an app installed elsewhere or a mounted DMG, set `PHOXAL_MUJOCO_LIBRARY` to its actual library file.
An explicit path takes precedence and is never silently replaced with another library.
The internal binding adaptation retains the upstream licenses and generates typed runtime symbol dispatch from the original ABI declaration.
The library stays loaded for all engine objects and their destruction.

## Open and run

```sh
phoxal-simulator
```

This checks the user-managed MuJoCo prerequisite and opens an idle desktop window.
No supervisor or participants start until a build directory and an explicit scene are selected.
Help and version remain native-library-free.

From robot-rover, the separate source-development command prepares and launches the robot:

```sh
cargo phoxal simulation simulation/scene.xml
```

The CLI owns Cargo and assembly; the simulator consumes the runnable directory and owns execution.
Desktop runs advance immediately in Realtime mode and have no arbitrary step-count limit.
Use `--paused` for paused startup and `--duration 10s` for an explicit time bound.

- **Pause / Run** freezes controlled simulation and resumes its normal scheduling, preserving logical time, admitted authority and logical leases.
- **Step** advances one native boundary while paused.
- **Reset** resets the current execution while paused, including runtime reset and source-state handling.
- **Stop** releases native authority and shuts down the supervisor and participants.
- **Restart** becomes available after shutdown and starts a fresh execution from the same prepared scene.
- Closing the window stops and joins the current execution.

The native viewport occupies the main area, with a compact robot/status/time header.
The Scene list contains native MuJoCo body names and model-local IDs, not inferred Phoxal participants.
Selected-body details distinguish fixed bodies, a body's own free joint, and articulated/attached bodies using native model tables.
At narrow window widths, Scene and selected-body details collapse above the viewport.
Boundary, generation and actuator counts live in Diagnostics.
Robot body evidence records the root free body's physical origin and world quaternion [w, x, y, z].
Both velocity vectors use world axes, with linear velocity at the body origin rather than its inertial center.
MuJoCo converts the body-local angular DOFs into copied world-axis boundary evidence.

- **Click** selects a native surface; a miss clears selection.
- **Right drag** or macOS secondary click-and-drag orbits the camera.
- **Shift-right drag / middle drag** pans in the camera plane.
- **Scroll / trackpad scroll** zooms; **Shift-scroll** pans.
- **Pinch** uses the zoom gesture delivered by the window backend.
- **Focus selected** frames the selected body's native bounds and descendants.
- **Default view** restores MuJoCo's model-framing camera.

**Primary drag** grabs the selected native body, with an acknowledged drag status and the selected identity in the inspector.
A drag starting on a visible descendant of an explicitly selected ancestor moves that named ancestor; otherwise it selects the actual picked body.
To reposition rover's free root, select base_link in Scene before grabbing its visible chassis or wheels.
The tool never silently promotes a selected attached child to a free ancestor.
While running, translation uses MuJoCo spring/damping perturbation force and its native moment-arm torque at normal integration boundaries, without assigning qpos.
While paused, only the selected body's own free joint is eligible, and active native weld/connect constraints on its subtree refuse the pose edit.
Paused translation resets that free joint's six velocity DOFs, preserves orientation and other joint/body velocities, forwards native state and refreshes the authoritative snapshot without advancing time.
Component observations reflect the edited world through the next normal accepted boundary, not a fabricated product receipt.
Step and Run consume the edited pose and finish the paused gesture; start another drag to apply running force.
Fixed bodies remain selectable but refuse manipulation.
**Escape**, release and focus loss cancel drag; leaving the viewport keeps a held drag coherent until release, focus loss or liveness expiry.
Unchanged held input renews a 250 ms wall-clock UI drag deadline; this deadline does not affect logical robot leases.
Pause, Step, Run from paused, camera/selection changes, Reset, Stop, restart, window close and authority loss clear transient drag state.
Pointer displacement is limited to two viewport heights per axis, and each world translation component is bounded to 100 meters.
The begin hit is resolved on its exact presented snapshot, converted to a body-local anchor and reattached to that body's current native pose to avoid a jump on moving bodies.
Ongoing updates use the pinned camera axes/scale, viewport height and gesture identity, rather than requiring every update to reference the newest rendered frame.
One pending begin, one replaceable update and a terminal gesture high-water mark bound retention and prevent stale updates from reviving an ended gesture.
The authoritative Scene checks liveness again immediately before integrating, including after a slow remote prepare.
[MuJoCo's perturbation implementation](https://github.com/google-deepmind/mujoco/blob/3.12.0/src/engine/engine_vis_interact.c) overwrites the selected body's force slots.
The simulator calculates that contribution separately, adds it to the saved external forces for one integration and restores the exact baseline afterward; unrelated body forces and qfrc_applied remain untouched.
It never calls the existing wrapper that clears every body's xfrc_applied.
Coasting after release or timeout is native momentum, not retained GUI force.
Reset restores the original authored scene.
Camera distance is bounded to 0.01 through 100000 meters and elevation to -89 through 89 degrees; nonfinite gestures are ignored.
Native scene selection uses [MuJoCo mjv_select](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjv-select) with the displayed viewport aspect, left/bottom normalized cursor coordinates, exact rendered camera and copied state.
The input adapter uses [egui zoom_delta](https://docs.rs/egui/0.36.2/egui/struct.InputState.html#method.zoom_delta) for backend-delivered pinch or synthetic zoom, without applying its scroll contribution twice.
These controls require actual desktop gesture qualification on the host; offscreen renders do not prove window or trackpad behavior.

Selection highlights the native body's descendants with a restrained temporary tint that preserves source shading and surface detail; selecting world highlights only world's own geoms.
The renderer restores every changed color/emission after readback, including failure, without editing source materials or authoritative physics.
One replaceable camera request and one replaceable scene request bound input retention, independently of the execution control queue.
The worker resolves picks on its native workspace after copying the exact presented frame state; egui owns no mutable native objects.
Execution/model/reset generation, frame serial and camera checks reject stale requests rather than querying a different view.
Reset clears selection and pending operations while preserving the current camera for the same model.
Stop/close discard pending selection operations, and Restart starts with a fresh model-default view.
Rendering retains one pending frame and one presented immutable view, never an unbounded history.
Narrow windows scroll the complete Scene/inspector/Diagnostics content in a bounded region while preserving a visible viewport; wide windows scroll the inspector independently.
Diagnostics retain only the latest primary pointer transition and worker drag acknowledgement to distinguish a held gesture from same-frame press/release.
This does not change drag authority, safety or liveness and is not a complete input history.

The worker owns **Realtime | Fast** and acknowledges changes through the bounded command channel.
Realtime paces completed boundaries toward one simulated second per active wall second.
Fast adds no intentional wall-clock pacing, but cannot make a slow native/runtime pipeline faster than its actual throughput.
Neither mode changes the physics quantum, logical timestamps, participant cadence, leases, or boundary work.
Rendering is independently limited to approximately 30 frames per second in both modes.

The header shows **Sim**, **Wall**, and recent **Speed** alongside the actual scene.
Wall counts execution and pacing, including single-step work, and excludes acquisition/setup and paused idle time.
Pause freezes logical time and active wall time while native authority watchdog renewal continues.
Resume and mode switches re-anchor pacing and recent-rate samples, so there is no catch-up debt from pause or the previous mode.
Reset and Restart clear timing counters.
Speed uses the trailing two active-wall seconds, with samples no more than once per 100 ms and the preceding sample retained to bracket the window.
It displays `--` while paused and during the first 100 ms after starting/resuming/changing mode; single steps leave it paused.
A slow host shows its achieved rate below 1x instead of skipping work to catch up.

Ordinary desktop viewing retains only the current native snapshot and one pending rendered frame, rather than accumulating a trajectory.
Finite headless qualification and explicit scenario runs retain every native boundary for terminal evidence.
Scenario evidence is never silently truncated.
Boundary/generation and current actuation diagnostics remain worker-owned.
Hardware actuation is outside this simulation workflow.

A prepared bundle can also be run directly:

```sh
phoxal-simulator run /path/to/scene.xml --build /path/to/build
```

The same runnable build can be launched by the supervisor for hardware or consumed by the simulator.
Simulation validates its native model and the compiled component contracts before starting any participant.
The authored instances and connections stay intact; the simulator implements selected components natively, and the supervisor launches the brain and services without starting physical drivers.
A command-owned native context and execution state live outside the build and are removed after cleanup.
The validated model and providers stay alive through execution, rather than reopening mutable scene resources after launch.

Use `--headless --duration 10s` for finite qualification.
Durations accept units such as `10s` and `250ms`.
Both presentations use the same native coordinator, authenticated public protocol, actuator admission, observation capture, and receiver receipts.
Unknown transitions or required process, capture, and delivery failures stop the run.
Scenario execution uses a prepared run specification through `--simulation-run`; its results are reported after owned process cleanup.
A nonzero supervisor exit or forced shutdown fails the simulation, and the terminal report records the same exit outcome as the command.

## Development and qualification

Ordinary source builds need no MuJoCo:

```sh
cargo build
cargo test --test runtime_loading
```

CI checks library-free command behavior and ABI refusal on macOS and Linux, deterministic resource/configuration tests, formatting, strict Clippy, and documentation.
Native host acceptance additionally needs the external qualified library and graphics environment:

```sh
PHOXAL_MUJOCO_LIBRARY=/path/to/library cargo test --bin phoxal-simulator
cargo phoxal scenario scenarios/forward_stop.rs
```

Pacing/accounting and retention tests also run without a native library:

```sh
cargo test --bin phoxal-simulator execution::timing::
cargo test --bin phoxal-simulator execution::tests::ordinary_history_is_constant_and_explicit_evidence_is_complete
```

The explicit real-worker qualification test needs an existing runnable build and scene, plus the native library.
It exercises the worker, actual supervisor/participants, acknowledgements, pause/step/reset, pacing modes, restart and channel-close cleanup; it is separate from visual desktop acceptance.

```sh
PHOXAL_QUALIFICATION_BUILD=/path/to/build PHOXAL_QUALIFICATION_SCENE=/path/to/scene.xml cargo test --bin phoxal-simulator desktop::tests::native_worker_pacing_controls_and_cleanup -- --ignored --nocapture
```

Native scene selection qualification uses the same external build and scene prerequisites.
It exercises copied-state selection, focus, reset fencing, Stop and fresh execution cleanup, with optional offscreen images.
Those images are native render evidence, not desktop window/gesture acceptance.

```sh
PHOXAL_QUALIFICATION_BUILD=/path/to/build PHOXAL_QUALIFICATION_SCENE=/path/to/scene.xml PHOXAL_QUALIFICATION_IMAGES=/tmp/native-scene-images cargo test --bin phoxal-simulator desktop::tests::native_worker_scene_selection_fences_and_cleanup -- --ignored --nocapture
```

CI and release qualification deliberately use `--locked` to check the committed application lockfile.
Releases use ordinary crates.io publication through release-plz.
Application package versions are independent of SDK versions; actual interface revisions and target determine compatibility.
