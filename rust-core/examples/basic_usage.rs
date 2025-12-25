/// Basic usage example: Feed IMU samples, get motion evidence
use trace_sensing::{
    ImuSample, MotionEvidencePipeline, PipelineConfig,
};

fn main() {
    println!("=== Trace Sensing Engine: Basic Example ===\n");

    // Create pipeline with default config (50Hz, 1-second windows)
    let config = PipelineConfig::default();
    let mut pipeline = MotionEvidencePipeline::new(config);

    // Simulate IMU data stream: user walking then stopping
    let motion_samples = vec![
        // Walking phase (0-2 seconds)
        (1000, [0.5, 0.1, -9.81], [0.05, 0.02, 0.0]),
        (1020, [0.6, 0.15, -9.81], [0.06, 0.03, 0.0]),
        (1040, [0.55, 0.12, -9.80], [0.05, 0.02, 0.0]),
        (1060, [0.52, 0.11, -9.79], [0.04, 0.01, 0.0]),
        (1080, [0.58, 0.14, -9.81], [0.06, 0.02, 0.0]),
        
        // Transition: slowing down (2-2.5 seconds)
        (2000, [0.3, 0.05, -9.81], [0.02, 0.00, 0.0]),
        (2020, [0.2, 0.02, -9.81], [0.01, 0.00, 0.0]),
        (2040, [0.1, 0.01, -9.81], [0.00, 0.00, 0.0]),
        (2060, [0.05, 0.00, -9.81], [0.00, 0.00, 0.0]),
        
        // Still phase (2.5-4 seconds)
        (2080, [0.0, 0.0, -9.81], [0.0, 0.0, 0.0]),
        (2100, [0.01, 0.0, -9.81], [0.0, 0.0, 0.0]),
        (2120, [0.0, 0.01, -9.81], [0.0, 0.0, 0.0]),
        (2140, [0.0, 0.0, -9.81], [0.0, 0.0, 0.0]),
    ];

    println!("Processing {} samples...\n", motion_samples.len());

    let mut window_count = 0;

    for (timestamp, accel, gyro) in motion_samples {
        let sample = ImuSample::new(timestamp, accel, gyro);

        if let Some(window) = pipeline.process_sample(&sample) {
            window_count += 1;
            print_window(&window, window_count);
        }
    }

    // Flush any remaining data
    if let Some(window) = pipeline.flush() {
        window_count += 1;
        print_window(&window, window_count);
    }

    println!("\n=== Summary ===");
    println!("Total windows emitted: {}", window_count);
    println!("Motion analysis complete.");
}

fn print_window(window: &trace_sensing::MotionEvidenceWindow, num: usize) {
    println!("\n--- Window {} ---", num);
    println!("Timestamp: {} - {}ms", window.start_ms, window.end_ms);
    println!("Duration: {:.0}ms", window.duration_ms());
    println!("Motion mode: {:?}", window.context_mode);
    println!("Confidence: {:.2}", window.confidence);
    println!("Sensor health: {:?}", window.sensor_health);
    println!("Validity: {:?}", window.validity_state);

    if !window.segments.is_empty() {
        println!("  Segments: {}", window.segments.len());
        for (i, seg) in window.segments.iter().enumerate() {
            println!(
                "    [{i}] {:?}: {:.2} confidence, {:.0}ms",
                seg.mode, seg.confidence, seg.duration_ms as f32
            );
        }
    }

    if !window.transitions.is_empty() {
        println!("  Transitions: {}", window.transitions.len());
        for (i, trans) in window.transitions.iter().enumerate() {
            println!(
                "    [{i}] {:?}: {:.2} confidence",
                trans.transition_type, trans.confidence
            );
        }
    }
}
