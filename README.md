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

## Run a robot project

From robot-rover:

```sh
cargo phoxal simulation project
```

This opens a desktop simulation and starts advancing its scene.
The simulator invokes the public `cargo phoxal build --simulation-scene simulation/scene.xml` command to prepare a temporary bundle, then owns its execution.
`cargo phoxal simulation` is a thin process proxy to this application's parser, streams, and exit status.
Use `--scene <file>` for another scene, `--steps <count>` for the finite bound, or `--paused` to open paused.
The default bound is 10,000 native steps.

- **Pause / Run** suspends and resumes the current execution.
- **Step** advances one native boundary while paused.
- **Reset** resets the current execution while paused, including runtime reset and source-state handling.
- **Stop** releases native authority and shuts down the supervisor and participants.
- **Restart** becomes available after shutdown and starts a fresh execution from the same prepared scene.
- Closing the window stops and joins the current execution.

The desktop shows simulation time, boundary, and generation alongside the actual scene.
Hardware actuation is outside this simulation workflow.

A prepared bundle can also be run directly:

```sh
phoxal-simulator run --bundle /path/to/bundle --scene /path/to/bundle/scene/scene.xml --desktop --steps 10000 --auto-run
```

Use `--headless` instead of `--desktop` for finite qualification.
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
cargo phoxal test forward_turn_stop -- --nocapture
```

CI and release qualification deliberately use `--locked` to check the committed application lockfile.
Releases use ordinary crates.io publication through release-plz.
Application package versions are independent of SDK versions; actual interface revisions and target determine compatibility.
