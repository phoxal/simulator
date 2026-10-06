//! The simulator's providable endpoint contract.
//!
//! Every stream the simulator can provide is declared once here in Rust,
//! private to this binary, and every payload is an SDK standard contract:
//! the shared robotics, motion, and component vocabulary the concrete
//! drivers' capability declarations generate. The simulator authors no
//! component-owned schemas of its own.

use phoxal::contracts::component::actuator::ActuatorCommand;
use phoxal::contracts::component::camera::{CameraFrame, DepthFrame};
use phoxal::contracts::component::encoder::EncoderSample;
use phoxal::contracts::component::gnss::GnssSample;
use phoxal::contracts::component::imu::{AccelerometerSample, GyroscopeSample, ImuSample};
use phoxal::contracts::component::range::RangeSample;
use phoxal::contracts::{Latest, Queue};

/// The simulator's providable endpoint contract.
#[phoxal::endpoints]
pub struct SimulatorApi {
    #[phoxal::output(projection = state, lease_ms = 100, max_bytes = 1024)]
    actuators: Latest<ActuatorCommand>,

    #[phoxal::output(max_items = 16, max_bytes = 8192)]
    encoder: Queue<EncoderSample>,

    #[phoxal::output(max_items = 16, max_bytes = 16_384)]
    imu: Queue<ImuSample>,

    #[phoxal::output(max_items = 16, max_bytes = 8192)]
    accelerometer: Queue<AccelerometerSample>,

    #[phoxal::output(max_items = 16, max_bytes = 8192)]
    gyroscope: Queue<GyroscopeSample>,

    #[phoxal::output(max_items = 4, max_bytes = 8_388_608)]
    left_mono: Queue<CameraFrame>,

    #[phoxal::output(max_items = 4, max_bytes = 8_388_608)]
    rgb: Queue<CameraFrame>,

    #[phoxal::output(max_items = 4, max_bytes = 8_388_608)]
    right_mono: Queue<CameraFrame>,

    #[phoxal::output(max_items = 4, max_bytes = 8_388_608)]
    depth: Queue<DepthFrame>,

    #[phoxal::output(max_items = 16, max_bytes = 512)]
    range: Queue<RangeSample>,

    #[phoxal::output(max_items = 8, max_bytes = 8192)]
    gnss: Queue<GnssSample>,
}
