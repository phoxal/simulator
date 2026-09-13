# Contributing

## License and authorship

Contributions use the license already declared by the files they change.
This application is licensed under AGPL-3.0-only; see [LICENSE](LICENSE).
You retain copyright on your contribution.

Every commit must include a [Developer Certificate of Origin](https://developercertificate.org/) `Signed-off-by` trailer matching its author.
Use `git commit -s` to add it.
Commit messages and pull request titles follow [Conventional Commits](https://www.conventionalcommits.org/).
Use the breaking marker for a change to the public launch or simulation protocol contract.

## Application ownership

The executable owns native scene composition, providers, rendering, and desktop/headless coordination.
Keep application behavior in private modules under `src/`.
Import public runtime and service contracts from their framework owners.
Robot-specific model, mount, and control policy belongs to the robot project.

## Verification

Follow [README.md](README.md) for the pinned native installation and platform requirements.
Run these checks before submitting a change:

```sh
cargo fmt --all --check
cargo test --locked --all-targets
cargo clippy --locked --all-targets -- -D warnings
```

Exercise changed behavior through a real bundle-backed run and retain the command, platform, artifact identity, and result.
Native rendering checks need a functioning CGL or EGL implementation.
Test failures and unavailable required contexts are failed qualification, not successful skips.
A source build does not establish packaged loading, desktop controls, or compatibility with independently released supervisors.
The macOS packaging script creates a locally signed artifact for host acceptance; it does not claim distribution signing or notarization.

Do not manually edit generated bindings or the changelog.
