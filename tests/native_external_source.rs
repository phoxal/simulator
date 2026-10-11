//! External-source qualification requires an explicit test-owned robot/input fixture.
#[cfg(not(external_source_qualification))]
#[test]
fn declared_external_prerequisites_are_required() {
    panic!(
        "use RUSTFLAGS=--cfg external_source_qualification and the README fixture prerequisites"
    );
}
include!("../src/main.rs");
