# Phoxal Simulator

Phoxal's MuJoCo application owns one native scene and controls a robot through its supervisor's authenticated public simulation protocol.
It is an independent Cargo workspace and executable.
The hardware supervisor and ordinary robot services do not depend on this application or initialize MuJoCo.

Desktop and finite headless execution use the same coordinator.
At each boundary, the coordinator obtains the accepted actuator cut, integrates one native quantum, captures due observations, and waits for their receiver-admission receipt.
Lost replies are reconciled using the exact retained request and receipt.
An unknown outcome or required process, capture, or delivery failure ends the run.

## Installation boundary

Normal robot projects do not depend on MuJoCo or this package.
`cargo-phoxal` owns downloading and verifying MuJoCo, building the matching registry release, retaining native licenses and provenance, and maintaining the user installation.
This repository owns only the native simulator application and assumes that its build has been given a valid MuJoCo library directory.
This keeps host-specific native libraries out of robot dependency graphs and gives each simulator release one inspectable native identity.

Normal users install and maintain the released application with:

```sh
cargo phoxal simulation install
cargo phoxal simulation status
```

Simulator developers may still build this repository from source and pass its executable with `--simulator`.

## Build from source

The initial native pairing is `mujoco-rs` 6.0.1 with MuJoCo 3.12.0.
Install that native distribution and configure its library directory before building:

```sh
export MUJOCO_DYNAMIC_LINK_DIR=/opt/mujoco-3.12.0/lib
export LD_LIBRARY_PATH="$MUJOCO_DYNAMIC_LINK_DIR"
cargo build --locked
```

On macOS, use `DYLD_LIBRARY_PATH` instead of `LD_LIBRARY_PATH`.
The loader directory must contain the versioned library as well as its unversioned linker name.
The official MuJoCo 3.12.0 disk image can be used without copying its contents into this repository:

```sh
mkdir -p .local-mujoco/lib
ln -s /Volumes/MuJoCo/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib \
  .local-mujoco/lib/libmujoco.dylib
export MUJOCO_DYNAMIC_LINK_DIR="$PWD/.local-mujoco/lib"
export DYLD_LIBRARY_PATH=/Volumes/MuJoCo/mujoco.framework/Versions/A
cargo build --locked --release
```

`.local-mujoco/` is local build state and must not be committed.
Offscreen capture uses CGL on macOS and EGL on Linux; Linux needs an EGL/OpenGL implementation such as Mesa.
A headless run creates no desktop window.

Packaging is not implemented in this repository.
`cargo phoxal simulation install` owns the self-contained macOS application and Linux native installation so source releases cannot drift from the user-facing installer.

## Run

From a robot project, the development command prepares the immutable bundle, launches its supervisor, and supplies the simulator's connection endpoint:

```sh
cargo phoxal simulation run simulation/scene.xml \
  --headless --steps 50
```

Use `--simulator /absolute/path/phoxal-simulator` only to inject a source-built executable.

Use `--desktop` for the viewport, Run/Pause, Step, Reset, and Stop controls.
Drag the viewport to orbit and scroll to zoom.
Desktop mode begins paused; reaching the finite bound pauses again so the result can be inspected.
Reset starts a fresh timeline and establishes its initial observation cut before another step.
The viewport uses a separate read-only observation workspace and does not drive sensor capture rates.

A direct run needs a prepared simulation bundle and an already running supervisor:

```sh
phoxal-simulator \
  --scene simulation/scene.xml --bundle /absolute/path/bundle \
  --connect unixsock-stream//tmp/phoxal-run/router.sock \
  --scope local --supervisor-id sim --run-id run \
  --headless --steps 50
```

Replace `--steps` with `--duration` for an exact duration that is an integral number of authored physics quanta.
The final JSON record contains the native and completed boundary evidence, actuator/provider bindings, and artifact provenance.
A user stop before the finite bound is reported as stopped, without claiming that the requested transitions completed.

Tooling can inspect native composition without connecting or integrating:

```sh
phoxal-simulator --probe --scene simulation/scene.xml \
  --bundle /absolute/path/probe-bundle --json --headless
```

## Scene and capability authoring

The scene supplies the physics quantum and a `robot_mount` site.
The robot supplies its MJCF model, component mount sites, and selected components.
Composition uses native attachment with deterministic namespaces.
Actuator messages use authored `component.capability` identities; only the application resolves those identities to native actuator names.
No component-specific native target selection belongs in the framework SDK.

Camera capabilities must agree with their compiled native resolution and vertical field of view.
RGB/mono frames use top-to-bottom rows; depth is geometric optical-axis distance encoded in millimeters, with zero for invalid or out-of-range pixels.
Geometric depth does not reproduce a physical stereo reconstruction pipeline.
Range uses a fixed finite-FOV ray set and the nearest valid hit.
Publication rates remain phase-aligned with logical time and emit explicit NotDue membership between captures.

A scene containing GNSS must declare its georeference:

```xml
<custom>
  <numeric name="phoxal_georeference" data="0 0 0 0 0 0 0" />
  <text name="phoxal_georeference_axes" data="ENU" />
  <text name="phoxal_georeference_datum" data="WGS84_ELLIPSOIDAL" />
</custom>
```

The seven numbers are latitude and longitude in degrees, ellipsoidal altitude in meters, local reference X/Y/Z in meters, and yaw in radians.
The example is a synthetic equatorial origin, not a measured deployment location.
Missing or invalid metadata refuses GNSS admission.
GNSS is qualified within 100 km of the scene reference; a capture outside that extent fails the run.

## Verification

```sh
cargo test --locked --workspace --all-targets
cargo clippy --locked --workspace --all-targets -- -D warnings
```

The native tests require the same dynamic-library configuration as the executable.
They cover exact phase recovery, authority fencing, deterministic cadence, native encoder/control behavior, georeference, and encoding.
Full robot, desktop, memory-budget, process-failure, and released-artifact acceptance remain separate required proofs.
