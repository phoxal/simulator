//! Portable, static application interfaces, independent of package releases.
use phoxal::artifact::application::*;
pub const CONTRACT: ApplicationContract = ApplicationContract {
    bundle: Some(BUNDLE_CONTRACT),
    launch: SIMULATOR_LAUNCH_CONTRACT,
    execution: None,
    simulation: Some(SIMULATION_PROTOCOL_CONTRACT),
    target: HOST_EXECUTION_TARGET,
};
#[used]
#[cfg_attr(target_os = "macos", unsafe(link_section = "__DATA,__phoxal_app"))]
#[cfg_attr(target_os = "linux", unsafe(link_section = ".phoxal_app"))]
static RECORD: [u8; APPLICATION_RECORD_BYTES] = encode_application_contract(&CONTRACT);
