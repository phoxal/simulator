# Application release orchestration

The simulator is distributed only as complete applications through GitHub
Releases; the crate is not published to crates.io. The single
`release.yml` workflow is manually dispatched, resolves the one Cargo.toml
version against the exact dispatched source commit, builds the available
targets, smokes each extracted application and prepares a draft release
that only a human publishes.

## Targets and runners

The build matrix uses current GitHub-hosted runners: `macos-15` for
`aarch64-apple-darwin`, `ubuntu-22.04` for `x86_64-unknown-linux-gnu` and
`ubuntu-22.04-arm` for `aarch64-unknown-linux-gnu`. Each runner builds its
own architecture natively. An Intel macOS build is feasible in principle
through the universal upstream engine and the `macos-15-intel` runner, but
no Intel artifact is built or advertised; nothing here claims one.

## Version, source and refusal

The release version comes only from `Cargo.toml` and the tag is
`phoxal-simulator-v<version>`. The workflow refuses to touch a tag whose
release already exists and is published, including the currently published
zero-asset `phoxal-simulator-v0.2.4`; another release requires a new
Cargo.toml version. A draft from the same source commit may be refreshed
(delete and recreate) for retry; a draft from a different source is
refused.

## Packaging, smoke and checks

Every build checks out the exact resolved commit, provisions the
checksum-pinned private MuJoCo engine, builds with `cargo build --release
--locked`, and assembles the application with the repository packaging
scripts: a macOS `.app` wrapped into a DMG with an Applications shortcut,
and a relocatable per-architecture Linux directory archive. Artifacts use
ad-hoc signatures only; there is no Developer ID, notarization, stapling,
Apple signing secret or notarization credential in the workflow.

Each target smokes the extracted application away from the checkout with loader and engine environment hints removed.
The `smoke` CLI command uses the existing application world owner to execute ten ordinary native Step/render boundaries of the bundled default scene without a GUI, robot or supervisor.
Removing the extracted private engine must then fail closed rather than use a checkout or system library.
The source-built engine root is moved before extraction, so its original link directory is unavailable to this check.
Final target checksums are validated and sealed into a flat `SHA256SUMS.txt` containing only the exact three user-downloadable filenames.
The workflow serializes releases and checks published/different-source refusal again before preparing the draft.
Only a same-source draft can be refreshed; a human publishes only after the workflow completes and the complete draft is reviewed.
Published release bytes are never replaced.

Version 0.2.5 is the app source-delivery candidate, selected after live release/tag inspection found 0.2.4 already published with zero assets and no 0.2.5 release or tag.
The version comes from Cargo.toml; no source-install publication or separate app version is introduced.
Pull-request CI validates the delivered source on macOS and Linux.
Release-profile target builds, draft preparation, uploaded/downloaded verification and human publication require the separate manual release workflow after source review and merge; source delivery does not establish those outcomes.

## Local validation

With the README native build/load prerequisites available, run
`cargo test --locked`, `cargo clippy --locked --all-targets -- -D
warnings` and `cargo doc --locked --no-deps`; CI runs the same commands on
both host systems. Use `actionlint` and `bash -n` on the workflows and
scripts. Platform certification, downloaded-artifact acceptance and Linux
graphical qualification are honest unverified status, not prerequisites of
this development-tool release path.
