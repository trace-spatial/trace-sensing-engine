//! Trace IMU Movement Engine
//!
//! A privacy-first motion evidence kernel that converts raw inertial sensor
//! streams into structured motion evidence windows.
//!
//! This is the entry point for standalone binaries. For library use, see lib.rs.

use trace_sensing::signal::SignalProcessor;
use trace_sensing::types::{ImuSample, MotionEvidenceWindow, MotionMode, SensorHealth};

fn main() {
    println!("Trace IMU Movement Engine v0.1.0");
    println!("Motion evidence extraction kernel\n");

    // Example 1: Signal processing (gravity separation)
    println!("=== Signal Processing Demo ===");
    let mut processor = SignalProcessor::new();

    let samples = vec![
        ImuSample::new(0, [0.0, 0.0, -9.81], [0.0, 0.0, 0.0]),
        ImuSample::new(10, [0.1, 0.0, -9.81], [0.0, 0.0, 0.0]),
        ImuSample::new(20, [0.2, 0.0, -9.81], [0.0, 0.0, 0.0]),
        ImuSample::new(30, [0.1, 0.0, -9.81], [0.0, 0.0, 0.0]),
    ];

    for sample in &samples {
        let processed = processor.process_sample(sample);
        println!(
            "t={}ms: linear_accel_mag={:.3} m/s², health={:?}",
            processed.timestamp_ms, processed.linear_accel_magnitude, processed.sensor_health
        );
    }

    // Example 2: Motion evidence window
    println!("\n=== Motion Evidence Window Demo ===");
    let mut window = MotionEvidenceWindow::new(
        1,
        0,
        5000,
        MotionMode::SteadyMotion,
        SensorHealth::Nominal,
        1,
    );

    window.add_segment(trace_sensing::types::MotionSegment::new(
        0,
        5000,
        MotionMode::SteadyMotion,
        5.2,
        0.05,
        0.95,
    ));

    println!(
        "Window ID: {}, Duration: {}ms, Valid: {}",
        window.window_id,
        window.duration_ms(),
        window.is_valid()
    );
}
