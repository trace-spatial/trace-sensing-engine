//! Trace IMU Movement Engine
//!
//! A privacy-first motion evidence kernel that converts raw inertial sensor
//! streams into structured motion evidence windows.
//!
//! This is the entry point for standalone binaries. For library use, see lib.rs.

pub mod types;

use types::{ImuSample, MotionEvidenceWindow, MotionMode, SensorHealth, ValidityState};

fn main() {
    // Placeholder for CLI interface
    println!("Trace IMU Movement Engine v0.1.0");
    println!("Motion evidence extraction kernel");

    // Example: create a simple motion evidence window
    let mut window = MotionEvidenceWindow::new(
        1,
        0,
        5000,
        MotionMode::SteadyMotion,
        SensorHealth::Nominal,
        1,
    );

    window.add_segment(types::MotionSegment::new(
        0,
        5000,
        MotionMode::SteadyMotion,
        5.2,
        0.05,
        0.95,
    ));

    println!("Created evidence window: id={}, duration={}ms", window.window_id, window.duration_ms());
    println!("Validity: {:?}", window.validity_state);
}
