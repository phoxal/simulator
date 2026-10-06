# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0](https://github.com/phoxal/simulator/compare/phoxal-simulator-v0.1.3...phoxal-simulator-v0.2.0) - 2026-10-06

### Added

- [**breaking**] run common builds with named native actuator routes

## [0.1.3](https://github.com/phoxal/simulator/compare/phoxal-simulator-v0.1.2...phoxal-simulator-v0.1.3) - 2026-10-04

### Fixed

- *(simulator)* resolve mesh extensions without ASCII case sensitivity
- *(simulator)* stabilize native model identity across restarts

## [0.1.2](https://github.com/phoxal/simulator/compare/phoxal-simulator-v0.1.1...phoxal-simulator-v0.1.2) - 2026-10-04

### Fixed

- *(simulator)* propagate supervisor shutdown failures

### Other

- *(simulator)* publish shutdown fixture readiness atomically
- *(simulator)* qualify clean public installs without MuJoCo

## [0.1.1](https://github.com/phoxal/simulator/compare/phoxal-simulator-v0.1.0...phoxal-simulator-v0.1.1) - 2026-10-04

### Fixed

- *(simulator)* discover standard macOS MuJoCo applications

### Other

- *(release)* update package versions ([#5](https://github.com/phoxal/simulator/pull/5))

## [0.1.0](https://github.com/phoxal/simulator/releases/tag/phoxal-simulator-v0.1.0) - 2026-10-04

### Added

- *(simulator)* load user-managed MuJoCo and own simulation execution
- [**breaking**] complete the native MuJoCo application
- *(simulator)* expose scenario motion evidence
- checkpoint independent MuJoCo simulator application

### Fixed

- *(simulator)* guard model admission and separate native qualification
- consume framework packages from registry
- *(simulator)* correct three scene.rs paths to canonical module locations
- *(simulator)* migrate bundle receiver imports to phoxal-artifact-format

### Other

- *(simulator)* repair adapted binding links and check both platforms
- Consume the published framework 0.0.0-dev.6 registry release ([#4](https://github.com/phoxal/simulator/pull/4))
- Consume the SDK actuator namespace and geometry vocabulary
- remove retained request reconciliation and retry logic
- consume build-script robot bundles
- Convert versioned schema discriminators to internally-tagged enums ([#3](https://github.com/phoxal/simulator/pull/3))
- [**breaking**] publish the MuJoCo application as phoxal-simulator ([#2](https://github.com/phoxal/simulator/pull/2))
- *(simulator)* track final Unit 6 framework HEAD (Unit 7)
- *(simulator)* receive phoxal-mujoco as private mujoco modules and rebaseline stale deps (Unit 6 revision)
- *(simulator)* rebase onto framework rename + consolidation
- *(.gitignore)* ignore local-only toolchain pin, MuJoCo shim, cargo config
