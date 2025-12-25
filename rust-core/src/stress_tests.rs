/// Production-level stress testing for the motion evidence pipeline.
/// 
/// These tests are designed to expose real-world failure modes that would only
/// appear under extreme, sustained, or pathological conditions.

#[cfg(test)]
mod stress_tests {
    use crate::types::*;
    use crate::pipeline::*;

    // ============================================================================
    // CATEGORY 1: EXTREME DURATION & THROUGHPUT
    // ============================================================================

    /// Test 1 minute of continuous processing at 50Hz (3000 samples)
    #[test]
    fn stress_one_minute_continuous_50hz() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        let mut window_count = 0;
        
        for sample_idx in 0..3000 {
            let timestamp = sample_idx as u64 * 20;
            
            let (accel_x, gyro_z) = if sample_idx % 1000 < 500 {
                (2.0, 0.1)
            } else {
                (0.1, 0.01)
            };
            
            let sample = ImuSample::new(
                timestamp,
                [accel_x, 0.5, 9.8],
                [0.05, 0.05, gyro_z],
            );
            
            if let Some(_window) = pipeline.process_sample(&sample) {
                window_count += 1;
            }
        }
        
        if let Some(_final) = pipeline.flush() {
            window_count += 1;
        }
        
        assert!(window_count > 0, "Must emit windows");
        assert!(window_count <= 65, "60s at 1000ms windows");
    }

    /// Test 10 minute marathon: Continuous wearable operation
    #[test]
    fn stress_ten_minute_marathon_100hz() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        let mut total_windows = 0;
        
        for i in 0..60_000 {
            let timestamp = i as u64 * 10;
            
            let pattern = (i / 1000) % 5;
            let (ax, ay, az, gx, gy, gz) = match pattern {
                0 => (0.1, 0.1, 9.8, 0.01, 0.01, 0.01),
                1 => (1.5, 0.2, 9.8, 0.05, 0.05, 0.05),
                2 => (0.5, 3.0, 9.8, 1.0, 0.1, 0.1),
                3 => (5.0, 2.0, 8.5, 0.2, 0.2, 0.3),
                _ => (-2.0, -1.5, 10.2, -0.1, -0.1, -0.2),
            };
            
            let sample = ImuSample::new(timestamp, [ax, ay, az], [gx, gy, gz]);
            
            if let Some(_) = pipeline.process_sample(&sample) {
                total_windows += 1;
            }
        }
        
        if let Some(_) = pipeline.flush() {
            total_windows += 1;
        }
        
        // Should produce many windows
        assert!(total_windows > 0, 
                "Expected windows, got {}", total_windows);
    }

    // ============================================================================
    // CATEGORY 2: EXTREME PARAMETER VALUES
    // ============================================================================

    /// Test with extreme accelerometer values (free fall, impact)
    #[test]
    fn stress_extreme_acceleration_values() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        let test_cases = vec![
            [0.0, 0.0, 0.0],
            [50.0, 50.0, 50.0],
            [-50.0, -50.0, -50.0],
            [999.9, 0.0, 0.0],
            [0.0, 999.9, 0.0],
            [0.0, 0.0, 999.9],
        ];
        
        for (idx, &accel) in test_cases.iter().enumerate() {
            let sample = ImuSample::new(
                idx as u64 * 100,
                accel,
                [0.0, 0.0, 0.0],
            );
            let _ = pipeline.process_sample(&sample);
        }
        
        let _ = pipeline.flush();
    }

    /// Test with extreme gyroscope values (rapid rotation)
    #[test]
    fn stress_extreme_gyro_values() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        let test_cases = vec![
            [50.0, 50.0, 50.0],
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            [0.0, 0.0, 100.0],
            [-50.0, -50.0, -50.0],
        ];
        
        for (idx, &gyro) in test_cases.iter().enumerate() {
            let sample = ImuSample::new(
                idx as u64 * 100,
                [0.1, 0.1, 9.8],
                gyro,
            );
            let _ = pipeline.process_sample(&sample);
        }
        
        let _ = pipeline.flush();
    }

    /// Test with NaN and Inf values (corrupted sensor data)
    #[test]
    fn stress_nan_and_inf_corruption() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        let pathological_cases = vec![
            [f32::NAN, 0.0, 0.0],
            [0.0, f32::NAN, 0.0],
            [0.0, 0.0, f32::NAN],
            [f32::INFINITY, 0.0, 0.0],
            [0.0, f32::INFINITY, 0.0],
            [0.0, 0.0, f32::INFINITY],
            [f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY],
        ];
        
        for (idx, &accel) in pathological_cases.iter().enumerate() {
            let sample = ImuSample::new(
                idx as u64 * 100,
                accel,
                [0.0, 0.0, 0.0],
            );
            let _ = pipeline.process_sample(&sample);
        }
        
        let _ = pipeline.flush();
    }

    // ============================================================================
    // CATEGORY 3: TEMPORAL CHAOS & STATE TRANSITIONS
    // ============================================================================

    /// Test rapid alternation between motion modes
    #[test]
    fn stress_rapid_mode_switching() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        for i in 0..5000 {
            let (ax, gy) = if i % 2 == 0 {
                (0.1, 0.01)
            } else {
                (10.0, 5.0)
            };
            
            let sample = ImuSample::new(
                i as u64 * 20,
                [ax, 0.2, 9.8],
                [0.1, 0.1, gy],
            );
            
            let _ = pipeline.process_sample(&sample);
        }
        
        let _ = pipeline.flush();
    }

    /// Test backward time (out-of-order timestamps)
    #[test]
    fn stress_nonmonotonic_timestamps() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        let timestamps = vec![
            0, 20, 40, 60, 80,
            100, 90, 80, 110,
            200, 150, 250, 160,
            300, 305, 301, 310,
        ];
        
        for (idx, &ts) in timestamps.iter().enumerate() {
            let sample = ImuSample::new(
                ts,
                [0.5 + (idx as f32 * 0.1), 0.2, 9.8],
                [0.0, 0.0, 0.0],
            );
            let _ = pipeline.process_sample(&sample);
        }
        
        let _ = pipeline.flush();
    }

    /// Test zero-time samples (duplicates or burst)
    #[test]
    fn stress_zero_time_deltas() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        for _ in 0..1000 {
            let sample = ImuSample::new(
                0,
                [1.0, 0.5, 9.8],
                [0.1, 0.1, 0.1],
            );
            let _ = pipeline.process_sample(&sample);
        }
        
        let _ = pipeline.flush();
    }

    // ============================================================================
    // CATEGORY 4: MEMORY & ALLOCATION STRESS
    // ============================================================================

    /// Test with extreme sample rates
    #[test]
    fn stress_extreme_sample_rate_variations() {
        // Minimum throughput
        let config_slow = PipelineConfig {
            sample_rate_hz: 1.0,
            window_duration_ms: 1000,
            ..Default::default()
        };
        let mut pipeline_slow = MotionEvidencePipeline::new(config_slow);
        
        for i in 0..100 {
            let sample = ImuSample::new(
                i as u64 * 1000,
                [0.5, 0.2, 9.8],
                [0.0, 0.0, 0.0],
            );
            let _ = pipeline_slow.process_sample(&sample);
        }
        let _ = pipeline_slow.flush();
        
        // Maximum throughput
        let config_fast = PipelineConfig {
            sample_rate_hz: 1000.0,
            window_duration_ms: 100,
            ..Default::default()
        };
        let mut pipeline_fast = MotionEvidencePipeline::new(config_fast);
        
        for i in 0..100_000 {
            let sample = ImuSample::new(
                i as u64,
                [0.5, 0.2, 9.8],
                [0.0, 0.0, 0.0],
            );
            let _ = pipeline_fast.process_sample(&sample);
        }
        let _ = pipeline_fast.flush();
    }

    // ============================================================================
    // CATEGORY 5: RECOVERY & RESILIENCE
    // ============================================================================

    /// Test recovery from pathological state
    #[test]
    fn stress_recovery_from_corrupted_state() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        // Normal operation
        for i in 0..500 {
            let sample = ImuSample::new(
                i as u64 * 20,
                [0.5, 0.2, 9.8],
                [0.0, 0.0, 0.0],
            );
            let _ = pipeline.process_sample(&sample);
        }
        
        // Corrupted data
        for i in 500..1000 {
            let sample = if i % 2 == 0 {
                ImuSample::new(
                    i as u64 * 20,
                    [f32::NAN, 999.0, f32::INFINITY],
                    [f32::NEG_INFINITY, 0.0, 0.0],
                )
            } else {
                ImuSample::new(
                    i as u64 * 20,
                    [50.0, -50.0, 50.0],
                    [100.0, 100.0, -100.0],
                )
            };
            let _ = pipeline.process_sample(&sample);
        }
        
        // Recovery
        let mut recovered_windows = 0;
        for i in 1000..1500 {
            let sample = ImuSample::new(
                i as u64 * 20,
                [0.5, 0.2, 9.8],
                [0.0, 0.0, 0.0],
            );
            if let Some(w) = pipeline.process_sample(&sample) {
                recovered_windows += 1;
                assert!(w.confidence >= 0.0 && w.confidence <= 1.0);
            }
        }
        
        if pipeline.flush().is_some() {
            recovered_windows += 1;
        }
        
        assert!(recovered_windows > 0, "Must recover to producing windows");
    }

    /// Test repeated flush cycles
    #[test]
    fn stress_repeated_flush_cycles() {
        let config = PipelineConfig::default();
        
        for cycle in 0..100 {
            let mut pipeline = MotionEvidencePipeline::new(config.clone());
            
            for i in 0..50 {
                let sample = ImuSample::new(
                    i as u64 * 20 + cycle as u64 * 1000,
                    [0.5 + (cycle as f32 * 0.01), 0.2, 9.8],
                    [0.0, 0.0, 0.0],
                );
                let _ = pipeline.process_sample(&sample);
            }
            
            let _ = pipeline.flush();
        }
    }

    // ============================================================================
    // CATEGORY 6: BOUNDARY CONDITIONS
    // ============================================================================

    /// Test extreme configurations
    #[test]
    fn stress_extreme_configurations() {
        // Minimum config
        let config_min = PipelineConfig {
            sample_rate_hz: 1.0,
            window_duration_ms: 1,
            ..Default::default()
        };
        let mut pipeline_min = MotionEvidencePipeline::new(config_min);
        
        for i in 0..100 {
            let sample = ImuSample::new(i as u64, [0.1, 0.1, 9.8], [0.0, 0.0, 0.0]);
            let _ = pipeline_min.process_sample(&sample);
        }
        let _ = pipeline_min.flush();
        
        // Maximum config
        let config_max = PipelineConfig {
            sample_rate_hz: 10_000.0,
            window_duration_ms: 1_000_000,
            ..Default::default()
        };
        let mut pipeline_max = MotionEvidencePipeline::new(config_max);
        
        for i in 0..100 {
            let sample = ImuSample::new(i as u64, [0.1, 0.1, 9.8], [0.0, 0.0, 0.0]);
            let _ = pipeline_max.process_sample(&sample);
        }
        let _ = pipeline_max.flush();
    }

    /// Test constant gravity input
    #[test]
    fn stress_constant_gravity_input() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        for i in 0..5000 {
            let sample = ImuSample::new(
                i as u64 * 20,
                [0.0, 0.0, 9.81],
                [0.0, 0.0, 0.0],
            );
            let _ = pipeline.process_sample(&sample);
        }
        
        let _ = pipeline.flush();
    }

    // ============================================================================
    // CATEGORY 7: PRODUCTION PATTERNS
    // ============================================================================

    /// Smartwatch realistic pattern
    #[test]
    fn stress_smartwatch_realistic_pattern() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        let mut sample_idx = 0;
        
        // Normal wrist movement (30s)
        for _ in 0..1500 {
            let sample = ImuSample::new(
                sample_idx * 20,
                [2.0 + (sample_idx as f32 * 0.001).sin(), 
                 1.0 + (sample_idx as f32 * 0.002).cos(),
                 9.8],
                [0.5, 0.3, 0.1],
            );
            let _ = pipeline.process_sample(&sample);
            sample_idx += 1;
        }
        
        // Arm drop (5s)
        for _ in 0..250 {
            let sample = ImuSample::new(
                sample_idx * 20,
                [15.0 - (sample_idx as f32 % 250.0) * 0.06, 10.0, 8.0],
                [3.0, 2.0, 0.5],
            );
            let _ = pipeline.process_sample(&sample);
            sample_idx += 1;
        }
        
        // Stillness (20s)
        for _ in 0..1000 {
            let sample = ImuSample::new(
                sample_idx * 20,
                [0.05, 0.05, 9.81],
                [0.01, 0.01, 0.01],
            );
            let _ = pipeline.process_sample(&sample);
            sample_idx += 1;
        }
        
        // Pickup and raise (10s)
        for _ in 0..500 {
            let sample = ImuSample::new(
                sample_idx * 20,
                [5.0 + (sample_idx as f32 * 0.01).sin(),
                 3.0,
                 9.8 + (sample_idx as f32 % 500.0) * 0.004],
                [1.0, 0.5, 0.2],
            );
            let _ = pipeline.process_sample(&sample);
            sample_idx += 1;
        }
        
        let _ = pipeline.flush();
    }

    /// Phone realistic pattern
    #[test]
    fn stress_phone_realistic_pattern() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        let mut sample_idx = 0;
        
        // Pocket mode
        for _ in 0..2000 {
            let noise = ((sample_idx as f32 * 0.1).sin() * 0.5) as f32;
            let sample = ImuSample::new(
                sample_idx * 20,
                [noise, noise * 0.5, 9.81 + noise * 0.1],
                [noise * 0.1, noise * 0.05, 0.0],
            );
            let _ = pipeline.process_sample(&sample);
            sample_idx += 1;
        }
        
        // Running (10s)
        for _ in 0..500 {
            let period = sample_idx as f32 % 100.0;
            let sample = ImuSample::new(
                sample_idx * 20,
                [3.0 * ((period / 100.0) * 6.28).sin(),
                 5.0 * ((period / 100.0 + 1.57) * 6.28).cos(),
                 9.8 + 1.5 * ((period / 100.0 + 3.14) * 6.28).sin()],
                [0.5, 0.3, 0.2],
            );
            let _ = pipeline.process_sample(&sample);
            sample_idx += 1;
        }
        
        // Sitting still
        for _ in 0..1000 {
            let sample = ImuSample::new(
                sample_idx * 20,
                [0.1, 0.1, 9.81],
                [0.01, 0.01, 0.01],
            );
            let _ = pipeline.process_sample(&sample);
            sample_idx += 1;
        }
        
        let _ = pipeline.flush();
    }

    // ============================================================================
    // CATEGORY 8: ULTIMATE STRESS TEST
    // ============================================================================

    /// Ultimate stress test: 5 minute marathon with chaos
    #[test]
    fn stress_ultimate_five_minute_chaos() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        
        for i in 0..15_000 {
            let segment = (i / 3000) % 5;
            
            let (ax, ay, az, gx, gy, gz) = match segment {
                0 => {
                    let s = (i as f32 * 0.1).sin();
                    (s, s * 0.5, 9.8, 0.1 * s, 0.05 * s, 0.0)
                },
                1 => {
                    let s = (i as f32 * 1.0).sin();
                    (s * 5.0, s * 3.0, 9.8, s, s * 0.5, 0.0)
                },
                2 => {
                    (50.0, -50.0, 10.0, 10.0, -10.0, 0.0)
                },
                3 => {
                    (0.001, 0.001, 9.8, 0.0001, 0.0001, 0.0)
                },
                _ => {
                    let chaos = ((i as f32).sin() * (i as f32).cos()) % 10.0;
                    (chaos, -chaos * 0.5, 9.8 + chaos * 0.1, chaos * 0.1, chaos * 0.05, 0.0)
                },
            };
            
            let timestamp = if i % 1000 == 0 && i > 0 {
                (i as u64 - 10) * 20
            } else {
                i as u64 * 20
            };
            
            let sample = ImuSample::new(
                timestamp,
                [ax, ay, az],
                [gx, gy, gz],
            );
            
            let _ = pipeline.process_sample(&sample);
            
            if i % 5000 == 4999 {
                let _ = pipeline.flush();
            }
        }
        
        let final_result = pipeline.flush();
        if let Some(w) = final_result {
            assert!(w.confidence >= 0.0 && w.confidence <= 1.0);
        }
    }
}
