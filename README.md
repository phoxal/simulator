# Phoxal Simulator

Native MuJoCo simulation and desktop controls for prepared Phoxal robots on Linux and macOS.
The simulator owns physics, rendering, its exact private engine and execution cleanup.
Robot source preparation belongs to cargo-phoxal.

## Application distribution

The application links upstream mujoco-rs 6.0.1 with exact MuJoCo 3.12.0.
The simulator is a development tool distributed only as complete applications through GitHub Releases; the crate is not published to crates.io and `cargo install` is not a supported journey.
A manually dispatched workflow builds the available targets from one exact source revision and prepares a draft release that a human publishes.
The workflow refuses any already published version, including releases with no assets; another release requires an unused version in `Cargo.toml`.
The macOS disk image contains the application with its private MuJoCo framework, metadata, notices and an Applications shortcut.
Linux distribution uses a relocatable per-architecture archive with the executable and its private engine; extract and run `bin/phoxal-simulator`, with a minimal desktop install recipe below.
Packaging uses `patchelf` on Linux to make the MuJoCo dependency a direct executable-relative path, so a missing private engine cannot silently fall back to a normal system installation.
Artifacts carry ad-hoc signatures only: there is no Developer ID, notarization or stapling pipeline, and platform certification, downloaded-artifact acceptance and Linux graphical qualification remain honestly unverified rather than release prerequisites.

A complete application needs no Cargo, separate runtime setup or first-run download to execute an already prepared compatible robot.
It does not discover a system MuJoCo, use a managed runtime cache or honor `PHOXAL_MUJOCO_LIBRARY`.
If its native engine is missing or damaged, the operating-system loader can prevent even help or the window from opening.
Reinstall the complete application; do not repair it by modifying the signed package or disabling operating-system security.
An exact ABI check precedes native model access and participant launch.

Launch the real executable in the package for developer commands.
For a macOS installation:

```sh
export PHOXAL_SIMULATOR='/Applications/Phoxal Simulator.app/Contents/MacOS/phoxal-simulator'
```

For a Linux installation, extract the relocatable archive and run the bundled executable directly:

```sh
tar -xzf phoxal-simulator-<version>-<arch>-linux.tar.gz -C ~/.local/opt
~/.local/opt/phoxal-simulator-<version>-<arch>-linux/bin/phoxal-simulator --version
```

A minimal desktop installation fills the absolute executable and icon paths into the shipped `share/phoxal-simulator.desktop.in` template and installs it where the desktop environment discovers `.desktop` files; no other setup exists or is needed.

Cargo-phoxal inspects that executable's embedded application contract, not the `.app` directory or a desktop shortcut.
Opening the application shows the bundled native default world immediately, with a 0.002 second physics quantum and its named robot attachment.
The initial world needs no robot build, supervisor, network or Cargo.
Run, Pause, Step, Reset and Stop also work on scenes without a robot.

Use Open scene to select an explicit confined MJCF resource closure, or New simulation to return to the exact bundled default.
Import robot selects a compatible prepared build and an eligible named world-level attachment.
Preview validates the complete native composition and bindings without changing the confirmed world; Place robot commits the displayed world-frame position and orientation without starting execution.
Cancel or failed admission keeps the confirmed world and authored state.
Single-robot import, removal and Undo require confirmed Stop, including while an execution is paused.
Changed manifest/model/component bytes between preview and Place, or before Run, require a fresh explicit import.
Original scene/build files are preserved.

Sessions are read-only: the simulator consumes complete prepared bundles and scene closures without modifying them, and there is no Save, saved-simulation Open or relink path.
Nothing about a session — motion, camera, selection, pacing or a session-only Remove/Undo — writes the bundle, the scene or any authored robot input.
Replacing the scene or robot waits for confirmed Stop and successful cleanup; Close requests cancellation and joins owned work while reporting any uncertainty.
Inspector and console are closed by default.
The fixed document bar and bottom corner controls share compact named icon actions, with larger targets available in Help.
Navigate is the default: primary drag orbits, Shift-primary/secondary/middle drag pans, and scroll or pinch zooms.
Manipulate is explicit; Escape, focus loss, overlays, tool changes, Reset and execution replacement cancel held gestures until release and a fresh press.
Follow tracks the verified native robot root or an explicitly selected movable native body, without changing physics; pan, Stop or replacement ends tracking.
The left outline toggle opens the native hierarchy, the scene menu opens the bounded resizable console, and Inspector exposes read-only Properties and Motion.
New explicit selection or successful placement reveals Inspector; deliberately closing it lasts until another explicit selection or placement.
The header visibly names the current scene; the placement sheet shows the selected prepared build before confirmation.
Help and placement modals trap keyboard focus and restore the invoking control.
Scene/bundle admission refusals are scoped notices that keep the current simulation and never mask its Running/Paused/Stopped state.
Native/offscreen input, layout and physics tests do not establish actual graphical or trackpad acceptance.

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
`probe <scene> --build <build> --json` returns native facts without participant launch.
`stage-scene --scene <scene> --output <directory>` preserves the closed resource bytes in the output directory and writes no success data to stdout.
Argument errors exit 2; admission/execution failures exit 1; completed or deliberately stopped execution exits 0 with the existing terminal evidence.
`smoke` executes ten ordinary native Step boundaries and renders the bundled default world without opening a window or requiring a robot build.
It is the extracted-application integrity check, and reports failure rather than GUI or controller acceptance.
An explicit developer invocation retains its original execution outcome and report through later session edits or runs.
Later cleanup uncertainty is reported separately and continues to fence execution/model changes; closing never confirms uncertain cleanup.

## Controls

- **Run / Pause:** resume or freeze logical time, native world, runtime state, admitted authority and logical leases.
- **Step:** one normal native boundary while paused.
- **Reset:** restore the original scene and reset the execution while paused.
- **Cancel startup / Stop:** cancel acquisition or release authority and shut down owned execution.
- **Restart:** fresh execution after confirmed cleanup.
- **Realtime / Fast:** paced or uncapped boundary execution, without changing quantum, cadence or leases.

Sim is logical time; Wall excludes paused idle and preparation; Speed is the recent achieved rate and shows `--` while paused or before enough active samples.
Slow hosts report their actual rate rather than skipping work.
Closing the window cancels and joins owned work.
Unsuccessful or unconfirmed authority/session/process cleanup disables restart and retains the primary cause and cleanup details.

Navigate is the default tool.
Click a native body or select its row in Outline; a miss clears selection, and the outline row label selects while its chevron only expands.
Primary drag orbits, Shift-primary/secondary/middle drag pans, scroll or pinch zooms, and Shift-scroll pans.
Frame selected frames the selected body; Default view restores the model camera.
Scene names/IDs belong to the native model, not inferred service identities.
For an attached child, Select robot body explicitly selects the nearest ancestor with its own free joint that movement and forces apply to.

Select Manipulate explicitly before primary dragging to apply bounded physical spring/damping force while running, without teleporting qpos.
Paused manipulation translates only an eligible body's own free joint, preserves orientation, resets its six velocity DOFs and forwards authoritative state without advancing time.
Fixed/constrained bodies refuse manipulation, while ordinary navigation and selection remain available.
Release, Escape, focus loss, modal/overlay capture, tool changes, pause, reset, stop and execution replacement terminate held gestures without clearing unrelated forces.
Canceled gestures require release and a fresh press before navigation or manipulation can resume.
A stationary held manipulation renews a 250 ms wall-clock UI deadline, independent of robot logical leases.
Reset restores the confirmed authored placement.

Robot body evidence refers to the physical root-body origin, not footprint or center of mass.
Position and both velocity vectors use world axes; orientation is body-to-world unit quaternion wxyz and linear velocity is at that origin.

## Troubleshooting

The visible notice shows what failed, why and the permitted next action; details retain exact phase/path, child stderr and cleanup outcome.
Invalid model/scene: repair the selected model/resources; reinstalling the application does not repair authored geometry.
Graphics failure: use a working graphical session or the headless command.
Unconfirmed shutdown: close the application and resolve the reported remote authority/session or process cleanup before reopening.
Closing does not confirm remote cleanup; same-process restart and model changes remain fenced.
Actual desktop and held-gesture acceptance requires a usable graphical host; offscreen renders and injected commands do not prove it.

## Contributor source builds

Contributor source builds retain native prerequisites for compilation and process loading; the application crate is not published to crates.io.
It is separate from the end-user application installation journey.
Use Rust 1.95 or newer and the platform graphics development libraries.
On GNU Linux, install EGL/OpenGL, xkbcommon and Wayland development packages through the host's package manager.
Provision the checksum-pinned official engine once into the repository-local `.local-mujoco` root using the repository helper:

```sh
target=$(rustc -vV | sed -n 's/^host: //p')
bash .github/scripts/provision-mujoco.sh "$target" "$(pwd)/.local-mujoco"
cargo build
cargo run --bin phoxal-simulator
cargo test --locked
cargo fmt --check
cargo clippy --all-targets -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps
```

The simulator is a development tool in its current scope: platform certification, signing/notarization, release installation and Linux graphics are unverified status, not prerequisites for building and running it from source.

The helper downloads and verifies MuJoCo once; it is never called by the application.
No environment exports are needed: the committed `.cargo/config.toml` resolves `MUJOCO_DYNAMIC_LINK_DIR` from the repository root for ordinary Cargo and IDE builds, and an explicitly exported value (as CI provisioning does) takes precedence over it.
Source-built executables record the selected link directory itself as their rpath, so direct runs, IDE runs, tests, examples and relocated or explicit-target `CARGO_TARGET_DIR` builds all load the engine without a terminal environment wrapper.
A missing provisioned engine fails the build with the exact expected directory instead of using a system installation.
A RustRover Run configuration needs nothing beyond the checkout: a Cargo run configuration with command `run --bin phoxal-simulator` and the working directory set to the simulator checkout root, with no environment overrides once the engine is provisioned.
Keep the provisioned root in place while running source-built binaries, including help, staging and non-physics tests.
`cargo test --locked` discovers the complete non-graphical application suite and executable-contract consumers.
It needs the linked engine for process loading, but does not execute native physics, renderers or real robot fixtures.
CI runs the same test, clippy and doc commands on Linux and macOS hosts.
Native qualification and actual desktop acceptance are separate.

The opt-in native target uses a test-only Rust configuration, not an application feature or alternate implementation:

```sh
RUSTFLAGS='--cfg native_qualification' \
PHOXAL_QUALIFICATION_BUILD=/absolute/prepared/build \
PHOXAL_QUALIFICATION_SCENE=/absolute/scene.xml \
PHOXAL_QUALIFICATION_FIXTURES=/absolute/test-owned/startup-fixtures \
cargo test --locked --test native_qualification -- --test-threads=1
```

It requires a working CGL context on macOS or EGL/OpenGL on Linux, the exact engine, a compatible prepared rover and test-owned startup fixtures.
Create the startup fixtures from an already prepared compatible rover and scene, from the simulator checkout.
Python 3 reads the admitted manifest's actual supervisor path; only newly created copies are changed.
The earlier source-build commands must have produced `target/debug/phoxal-simulator` with the engine available.

```sh
export PHOXAL_QUALIFICATION_BUILD=/absolute/prepared/build
export PHOXAL_QUALIFICATION_SCENE=/absolute/scene.xml
mkdir -p target
export PHOXAL_QUALIFICATION_FIXTURES="$(mktemp -d "$PWD/target/native-fixtures.XXXXXX")"
target/debug/phoxal-simulator stage-scene --scene "$PHOXAL_QUALIFICATION_SCENE" \
  --output "$PHOXAL_QUALIFICATION_FIXTURES"
for kind in held failure; do
  cp -R "$PHOXAL_QUALIFICATION_BUILD/." "$PHOXAL_QUALIFICATION_FIXTURES/$kind-build"
  rustc --edition=2024 "tests/fixtures/process/supervisor_$kind.rs" \
    -o "$PHOXAL_QUALIFICATION_FIXTURES/supervisor-$kind"
  python3 - "$PHOXAL_QUALIFICATION_FIXTURES" "$kind" <<'PYTHON'
import json, pathlib, shutil, sys
fixtures, kind = pathlib.Path(sys.argv[1]), sys.argv[2]
build = (fixtures / (kind + "-build")).resolve()
relative = pathlib.Path(json.loads((build / "manifest.json").read_text())["supervisor"]["path"])
assert not relative.is_absolute() and ".." not in relative.parts
supervisor = (build / relative).resolve(strict=True)
assert supervisor.is_relative_to(build)
shutil.copy2(fixtures / ("supervisor-" + kind), supervisor)
PYTHON
done
RUSTFLAGS='--cfg native_qualification' \
  cargo test --locked --test native_qualification -- --test-threads=1
```

The fixture directory contains staged `scene.xml` and its resources, plus `held-build` and `failure-build` copies whose supervisor executable alone is replaced.
This target retains native model, provider, admission, offscreen interaction, real-worker pacing and cleanup assertions.
It also exercises a held boundary acknowledgement and the same out-of-queue Stop latch used by desktop.
Missing prerequisites fail explicitly; qualification tests are not ignored.
These injected/offscreen tests do not establish visible GUI input, gestures or native window Close.

The separate `native_external_source` target retains the existing external-input pause trace campaign.
Invoke it with `RUSTFLAGS='--cfg external_source_qualification'` and explicit `PHOXAL_QUALIFICATION_{BUILD,SCENE,INPUT,CASES,TRACE_DIR,NEUTRAL,ENGAGED,ACTIVE}` values for a test-owned robot/input fixture.
The fixture must demonstrably produce nonzero native actuation before pause assertions; an absent or inert source is a qualification failure.
This test-owned source campaign does not establish physical controller acceptance.

No binding viewer/renderer feature is enabled: Phoxal retains its thread-owned CGL/EGL context and uses upstream rendering wrappers directly.

To assemble a private macOS proof, provide a new absolute `.app` destination, the exact build-source archive SHA256 and its base revision:

```sh
version=$(cargo metadata --no-deps --format-version 1 | python3 -c 'import json,sys; print(next(p["version"] for p in json.load(sys.stdin)["packages"] if p["name"] == "phoxal-simulator"))')
source_sha256=$(git archive --format=tar.gz HEAD | shasum -a 256 | cut -d' ' -f1)
bash .github/scripts/package-macos.sh target/debug/phoxal-simulator "$(pwd)/.local-mujoco" \
  '/absolute/new/Phoxal Simulator.app' "$version" "$(git rev-parse HEAD)" "$source_sha256"
bash .github/scripts/create-dmg.sh '/absolute/new/Phoxal Simulator.app' '/absolute/new/Phoxal Simulator.dmg'
```

This creates ad-hoc development signatures only, with no Developer ID release.
The `.github/workflows/release.yml` application-release workflow performs the same packaging from an exact clean revision, smokes the extracted application without the checkout engine, records final SHA-256 checksums and prepares a draft GitHub Release for human publication.
An Intel macOS application is feasible in principle through the universal upstream engine, but no Intel build exists and none is advertised.

## License

AGPL-3.0-only. See [LICENSE](LICENSE).

For changed or incompatible robot inputs, run `cargo phoxal build` from the original robot project, select its complete compatible prepared build, and explicitly import a new arrangement.
`cargo phoxal prepare` only snapshots contracts and composition; it does not assemble that runnable closure.

Release orchestration, qualification gates and retained finalized-byte retry are documented in [.github/README.md](.github/README.md).
Production assembly validates clean release identity and records `working_tree=clean-release`; local application/DMG qualification remains `uncommitted-development`.
