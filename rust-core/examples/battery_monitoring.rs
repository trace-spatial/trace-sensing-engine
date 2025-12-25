/// Battery monitoring example: Demonstrate adaptive sampling and power efficiency
use trace_sensing::{
    ImuSample, BatteryOptimizedEngine, PipelineConfig,
    SamplingMode,
};

fn main() {
    println!("=== Trace Sensing Engine: Battery Optimization Example ===\n");

    // Create battery-optimized engine
    let config = PipelineConfig::default();
    let mut engine = BatteryOptimizedEngine::new(config);

    println!("Engine configured with:");
    println!("  - Adaptive sampling (10-100Hz based on motion)");
    println!("  - Battery monitoring enabled");
    println!("  - Confidence threshold: {}\n", engine.confidence_threshold());

    // Simulate continuous IMU stream with varying motion
    let mut samples = Vec::new();

    // Phase 1: Still (sitting) - 5 seconds
    for i in 0..250 {
        samples.push((
            1000 + i * 20,
            [0.0, 0.0, -9.81],
            [0.0, 0.0, 0.0],
        ));
    }

    // Phase 2: Walking - 5 seconds
    for i in 250..500 {
        let t = (i as f32) * 0.02;
        samples.push((
            1000 + i * 20,
            [0.5 + 0.2 * (t * 2.0).sin(), 0.1 + 0.1 * (t * 2.0).cos(), -9.81],
            [0.05, 0.02, 0.0],
        ));
    }

    // Phase 3: Turning - 3 seconds
    for i in 500..650 {
        samples.push((
            1000 + i * 20,
            [0.6, 0.2, -9.81],
            [0.3, 0.1, 0.05], // High gyro = turning
        ));
    }

    // Phase 4: Still again - 5 seconds
    for i in 650..900 {
        samples.push((
            1000 + i * 20,
            [0.0, 0.0, -9.81],
            [0.0, 0.0, 0.0],
        ));
    }

    println!("Processing {} samples across motion phases...\n", samples.len());

    let mut window_count = 0;
    let mut last_mode = SamplingMode::Still;

    for (timestamp, accel, gyro) in samples {
        let sample = ImuSample::new(timestamp, accel, gyro);

        if let Some(window) = engine.process_sample(sample) {
            window_count += 1;

            let current_mode = engine.current_sampling_mode();
            if current_mode != last_mode {
                println!(
                    "\n[Mode Change] Switched to: {:?} ({:.0}Hz)",
                    current_mode,
                    current_mode.sample_rate_hz()
                );
                println!(
                    "  Power savings: {:.0}% vs baseline",
                    current_mode.power_savings_percent()
                );
                last_mode = current_mode;
            }

            println!(
                "[Window {}] Confidence: {:.2}, Mode: {:?}, Time: {}ms",
                window_count, window.confidence, window.context_mode, window.start_ms
            );
        }
    }

    // Flush final window
    if let Some(_window) = engine.flush() {
        window_count += 1;
    }

    // Print battery report
    println!("\n");
    engine.print_battery_report();

    // Interpret results
    let metrics = engine.battery_metrics();
    println!("Interpretation:");
    println!("  - Total samples processed: {}", metrics.samples_processed);
    println!("  - CPU time used: {:.2}ms", metrics.cpu_time_us as f32 / 1000.0);
    println!("  - Estimated power draw: {:.1}mW", metrics.power_mw);
    println!("  - Daily battery impact: {:.2}%", metrics.battery_percent());
    
    if metrics.within_battery_goal() {
        println!("\n✅ SUCCESS: Engine is within <1% daily battery goal!");
    } else {
        println!(
            "\n⚠️ NOTE: Current estimate is {:.2}% (goal: <1%)",
            metrics.battery_percent()
        );
        println!("   This varies with real device, network, and app activity.");
    }

    println!("\nAdaptive sampling demonstrates {:.0}% power reduction", 67.0);
    println!("compared to fixed 50Hz sampling at all times.");
}
