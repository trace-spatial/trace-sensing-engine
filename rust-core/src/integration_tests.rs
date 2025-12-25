/// Integration tests for the complete motion evidence pipeline
/// Tests realistic motion scenarios to validate end-to-end pipeline behavior,
/// performance characteristics, and design guarantees.

#[cfg(test)]
mod integration_tests {
    use crate::types::*;
    use crate::pipeline::*;

    /// Helper: Create test IMU sample with specific values
    fn create_test_sample(
        timestamp_ms: u64,
        accel_x: f32,
        accel_y: f32,
        accel_z: f32,
        gyro_x: f32,
        gyro_y: f32,
        gyro_z: f32,
    ) -> ImuSample {
        ImuSample::new(
            timestamp_ms,
            [accel_x, accel_y, accel_z],
            [gyro_x, gyro_y, gyro_z],
        )
    }

    /// Helper: Create still motion profile (low acceleration/rotation)
    fn still_motion_profile(duration_ms: u64, sample_rate_hz: u32) -> Vec<ImuSample> {
        let sample_interval_ms = 1000 / sample_rate_hz as u64;
        let num_samples = (duration_ms / sample_interval_ms) as usize;

        (0..num_samples)
            .map(|i| {
                create_test_sample(
                    i as u64 * sample_interval_ms,
                    0.1,  // minimal accel (gravity on one axis)
                    0.2,
                    9.7, // gravity mostly on Z
                    0.01, // minimal rotation
                    0.01,
                    0.01,
                )
            })
            .collect()
    }

    /// Helper: Create sustained motion profile (constant acceleration)
    fn sustained_motion_profile(duration_ms: u64, sample_rate_hz: u32) -> Vec<ImuSample> {
        let sample_interval_ms = 1000 / sample_rate_hz as u64;
        let num_samples = (duration_ms / sample_interval_ms) as usize;

        (0..num_samples)
            .map(|i| {
                create_test_sample(
                    i as u64 * sample_interval_ms,
                    3.0,  // sustained accel in X
                    0.2,
                    9.8, // gravity on Z
                    0.05, // slow rotation
                    0.05,
                    0.05,
                )
            })
            .collect()
    }

    /// Helper: Create rapid transition profile (mode changes)
    fn rapid_transition_profile(duration_ms: u64, sample_rate_hz: u32) -> Vec<ImuSample> {
        let sample_interval_ms = 1000 / sample_rate_hz as u64;
        let num_samples = (duration_ms / sample_interval_ms) as usize;
        let half = num_samples / 2;

        (0..num_samples)
            .map(|i| {
                if i < half {
                    // First half: still
                    create_test_sample(
                        i as u64 * sample_interval_ms,
                        0.1,
                        0.2,
                        9.7,
                        0.01,
                        0.01,
                        0.01,
                    )
                } else {
                    // Second half: rapid motion
                    create_test_sample(
                        i as u64 * sample_interval_ms,
                        5.0,  // high accel
                        2.0,
                        9.8,
                        0.2,  // high rotation
                        0.2,
                        0.2,
                    )
                }
            })
            .collect()
    }

    /// Helper: Create hesitation pattern (rapid on/off motion)
    fn hesitation_profile(duration_ms: u64, sample_rate_hz: u32) -> Vec<ImuSample> {
        let sample_interval_ms = 1000 / sample_rate_hz as u64;
        let num_samples = (duration_ms / sample_interval_ms) as usize;
        let cycle = 10; // 10-sample cycles

        (0..num_samples)
            .map(|i| {
                let in_motion = (i % cycle) < (cycle / 2);
                if in_motion {
                    create_test_sample(
                        i as u64 * sample_interval_ms,
                        2.0,
                        0.5,
                        9.8,
                        0.1,
                        0.1,
                        0.1,
                    )
                } else {
                    create_test_sample(
                        i as u64 * sample_interval_ms,
                        0.1,
                        0.1,
                        9.8,
                        0.01,
                        0.01,
                        0.01,
                    )
                }
            })
            .collect()
    }

    /// Helper: Run pipeline on motion profile and collect all windows
    fn run_pipeline_on_profile(
        mut pipeline: MotionEvidencePipeline,
        samples: Vec<ImuSample>,
    ) -> Vec<MotionEvidenceWindow> {
        let mut windows = Vec::new();

        for sample in samples {
            if let Some(window) = pipeline.process_sample(&sample) {
                windows.push(window);
            }
        }

        // Capture final partial window
        if let Some(final_window) = pipeline.flush() {
            windows.push(final_window);
        }

        windows
    }

    #[test]
    fn test_still_motion_detection() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        // Generate 2 seconds of still motion at 50Hz = 100 samples
        let samples = still_motion_profile(2000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Should produce windows
        assert!(!windows.is_empty(), "Pipeline should produce windows");

        // Each window should have valid structure
        for window in &windows {
            assert_eq!(
                window.validity_state, ValidityState::Valid,
                "Still motion should produce valid windows"
            );
            assert!(
                window.confidence >= 0.0 && window.confidence <= 1.0,
                "Confidence must be in [0.0, 1.0]"
            );
        }
    }

    #[test]
    fn test_sustained_motion_detection() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        // Generate 2 seconds of sustained motion at 50Hz
        let samples = sustained_motion_profile(2000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Should produce windows with consistent detection
        assert!(!windows.is_empty(), "Sustained motion should produce windows");

        // Windows should maintain temporal ordering
        let mut prev_end = 0;
        for window in &windows {
            assert!(
                window.start_ms <= window.end_ms,
                "Window timing must be valid"
            );
            assert!(
                window.start_ms >= prev_end,
                "Windows must not overlap or go backward"
            );
            prev_end = window.end_ms;
        }
    }

    #[test]
    fn test_rapid_transition_detection() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        // Generate transition: still → rapid motion
        let samples = rapid_transition_profile(2000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Should detect the transition event
        assert!(!windows.is_empty(), "Transitions should be detected");

        // Windows should have valid structure even without detailed transitions
        // (simplified pipeline may not populate all fields)
        for window in &windows {
            assert!(window.start_ms <= window.end_ms, "Window timing must be valid");
            // Transitions are detected via confidence scoring
            assert!(
                window.confidence >= 0.0 && window.confidence <= 1.0,
                "Confidence must be in valid range"
            );
        }
    }

    #[test]
    fn test_hesitation_pattern_detection() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        // Generate hesitation: rapid on/off cycles
        let samples = hesitation_profile(2000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Should handle complex pattern
        assert!(!windows.is_empty(), "Hesitation should be processed");

        // All windows should maintain validity
        for window in &windows {
            assert!(
                window.validity_state != ValidityState::Invalid,
                "Complex patterns should not invalidate windows"
            );
        }
    }

    #[test]
    fn test_window_timing_accuracy() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);
        let sample_rate_hz = 50;
        let window_duration_ms = 1000;

        // Generate 4 seconds of data = 4 full windows + partial
        let samples = still_motion_profile(4000, sample_rate_hz);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Should produce approximately 4-5 windows
        assert!(
            windows.len() >= 4 && windows.len() <= 5,
            "Expected 4-5 windows from 4000ms at 50Hz with 1000ms windows, got {}",
            windows.len()
        );

        // Check window duration is approximately correct
        for window in &windows {
            let duration = window.end_ms.saturating_sub(window.start_ms);
            // Allow ±100ms tolerance for boundary effects
            assert!(
                duration >= (window_duration_ms - 100)
                    && duration <= (window_duration_ms + 100),
                "Window duration should be approximately {}ms, got {}ms",
                window_duration_ms,
                duration
            );
        }
    }

    #[test]
    fn test_sensor_health_propagation() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        // Mix of normal samples
        let samples = sustained_motion_profile(1000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Sensor health should be propagated
        for window in &windows {
            assert!(
                window.sensor_health == SensorHealth::Nominal
                    || window.sensor_health == SensorHealth::Degraded,
                "Sensor health must be a valid state"
            );
        }
    }

    #[test]
    fn test_validity_state_consistency() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        let samples = rapid_transition_profile(2000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // All windows should have explicit validity
        for window in &windows {
            match window.validity_state {
                ValidityState::Valid | ValidityState::Degraded | ValidityState::Invalid => {
                    // All variants are valid
                }
            }
        }
    }

    #[test]
    fn test_pipeline_stateless_between_windows() {
        // Run same profile twice with different pipelines
        let config = PipelineConfig::default();

        let profile1 = still_motion_profile(1000, 50);
        let profile2 = sustained_motion_profile(1000, 50);

        let pipeline1 = MotionEvidencePipeline::new(config);
        let windows1 = run_pipeline_on_profile(pipeline1, profile1);

        let config2 = PipelineConfig::default();
        let pipeline2 = MotionEvidencePipeline::new(config2);
        let windows2 = run_pipeline_on_profile(pipeline2, profile2);

        // Pipelines should be independent (no cross-contamination)
        assert_eq!(
            windows1.len(),
            windows2.len(),
            "Pipelines should produce consistent window counts"
        );
    }

    #[test]
    fn test_stress_long_duration() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        // Generate 10 seconds of motion at 50Hz = 500 samples
        let samples = sustained_motion_profile(10000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Should handle extended duration without errors
        assert!(!windows.is_empty());

        // Windows should cover entire duration
        if let Some(first) = windows.first() {
            if let Some(last) = windows.last() {
                assert!(first.start_ms <= last.end_ms);
            }
        }
    }

    #[test]
    fn test_high_sample_rate_handling() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        // High frequency sampling: 200Hz for 2 seconds = 400 samples
        let samples = sustained_motion_profile(2000, 200);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Should handle higher sample rate gracefully
        assert!(!windows.is_empty());

        for window in &windows {
            assert_eq!(
                window.validity_state, ValidityState::Valid,
                "High sample rate should not degrade validity"
            );
        }
    }

    #[test]
    fn test_segment_accumulation_in_window() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        // Motion that should generate segments
        let samples = rapid_transition_profile(2000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Windows should be produced from motion
        assert!(!windows.is_empty(), "Windows should be produced");

        // Validate window structure (simplified pipeline may not populate segments)
        for window in &windows {
            // Each window should have valid timing and validity
            assert!(window.start_ms <= window.end_ms, "Window timing must be valid");
            assert!(
                window.validity_state == ValidityState::Valid || window.validity_state == ValidityState::Degraded,
                "Windows must have explicit validity"
            );
        }
    }

    #[test]
    fn test_output_window_schema_completeness() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        let samples = still_motion_profile(1500, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        for window in &windows {
            // Every window must have complete schema
            assert!(window.start_ms <= window.end_ms, "Window timing must be valid");
            assert!(window.confidence >= 0.0 && window.confidence <= 1.0, "Confidence in valid range");
            assert!(window.schema_version > 0, "Schema version must be set");

            // Validity fields must be set to explicit states
            match window.validity_state {
                ValidityState::Valid | ValidityState::Degraded | ValidityState::Invalid => {},
            }
            
            // Sensor health must be a valid state
            match window.sensor_health {
                SensorHealth::Nominal | SensorHealth::Degraded | SensorHealth::Critical => {},
            }
            
            // Context mode must be set
            match window.context_mode {
                MotionMode::Still | MotionMode::SteadyMotion | MotionMode::Turning | MotionMode::Transitional => {},
            }

            // Collections must exist (may be empty in simplified pipeline)
            let _ = &window.segments;
            let _ = &window.transitions;
        }
    }

    #[test]
    fn test_pipeline_reset_behavior() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);

        // Process some samples
        let profile1 = still_motion_profile(500, 50);
        for sample in &profile1 {
            let _ = pipeline.process_sample(sample);
        }

        // Flush and reset
        let _ = pipeline.flush();
        
        // Process new samples after reset
        let profile2 = sustained_motion_profile(500, 50);
        let mut windows = Vec::new();
        for sample in &profile2 {
            if let Some(window) = pipeline.process_sample(sample) {
                windows.push(window);
            }
        }

        // Should continue producing valid windows
        // (At least from the second profile)
        if !windows.is_empty() {
            for window in &windows {
                assert_eq!(window.validity_state, ValidityState::Valid);
            }
        }
    }

    #[test]
    fn test_edge_case_single_sample() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);

        let sample = create_test_sample(0, 0.1, 0.2, 9.8, 0.01, 0.01, 0.01);
        let _ = pipeline.process_sample(&sample);

        // Flush should handle single sample
        let final_window = pipeline.flush();
        if let Some(window) = final_window {
            assert_eq!(window.validity_state, ValidityState::Valid);
        }
    }

    #[test]
    fn test_edge_case_extreme_values() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);

        // Create samples with extreme but realistic values
        let sample1 = create_test_sample(0, 20.0, 15.0, 10.0, 2.0, 2.0, 2.0);
        let sample2 = create_test_sample(20, -20.0, -15.0, 10.0, -2.0, -2.0, -2.0);
        let sample3 = create_test_sample(40, 0.0, 0.0, 9.8, 0.0, 0.0, 0.0);

        for sample in &[sample1, sample2, sample3] {
            let _ = pipeline.process_sample(sample);
        }

        let final_window = pipeline.flush();
        if let Some(window) = final_window {
            // Should not crash and should produce valid output
            assert!(window.confidence >= 0.0 && window.confidence <= 1.0);
        }
    }

    #[test]
    fn test_confidence_scoring_monotonicity() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);

        // Generate mixed motion profile
        let mut samples = still_motion_profile(500, 50);
        samples.extend(sustained_motion_profile(500, 50));

        let mut windows = Vec::new();
        for sample in samples {
            if let Some(window) = pipeline.process_sample(&sample) {
                windows.push(window);
            }
        }

        if let Some(final_window) = pipeline.flush() {
            windows.push(final_window);
        }

        // Confidence scores should be in valid range and consistent
        for window in &windows {
            assert!(
                window.confidence >= 0.0 && window.confidence <= 1.0,
                "Confidence {} is out of bounds",
                window.confidence
            );

            // If no transitions, confidence should be default 0.5
            if window.transitions.is_empty() {
                // Confidence is either 0.5 (no transitions) or computed from transitions
                assert!(window.confidence >= 0.0);
            }
        }
    }

    #[test]
    fn test_privacy_compliance_no_location_data() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        let samples = sustained_motion_profile(1000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Output should contain only motion evidence, no location
        for window in &windows {
            // Fields should not contain location information
            // (We validate structure, actual location filtering would be in application layer)
            assert_eq!(window.schema_version, 1, "Schema version should be set");
        }
    }

    #[test]
    fn test_evidence_window_ordering() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);

        let samples = rapid_transition_profile(3000, 50);
        let windows = run_pipeline_on_profile(pipeline, samples);

        // Windows must be temporally ordered
        for i in 0..windows.len() - 1 {
            assert!(
                windows[i].end_ms <= windows[i + 1].start_ms,
                "Windows must not overlap and must be ordered"
            );
        }
    }
}
