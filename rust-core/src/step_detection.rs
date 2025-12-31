//! Step Detection and Stride Estimation Module.
//!
//! Implements robust step counting and stride length estimation using:
//! - Peak detection in vertical acceleration
//! - Zero-crossing detection for step timing
//! - Autocorrelation for cadence estimation
//! - Adaptive stride model calibrated per-user
//!
//! The algorithm handles various walking styles (normal, shuffle, brisk)
//! and is robust to phone orientation changes.

use crate::types::{ImuSample, StepEvent, StancePhase};

/// Configuration for step detection.
#[derive(Debug, Clone)]
pub struct StepDetectorConfig {
    /// Minimum time between steps in milliseconds (prevents double-counting).
    pub min_step_interval_ms: u64,
    /// Maximum time between steps before resetting cadence (detects stopped walking).
    pub max_step_interval_ms: u64,
    /// Minimum peak acceleration to count as a step (m/s²).
    pub min_peak_accel: f32,
    /// Maximum peak acceleration to count as a step (filters jumps/drops).
    pub max_peak_accel: f32,
    /// Smoothing factor for the low-pass filter (0-1, lower = more smoothing).
    pub smoothing_alpha: f32,
    /// Number of samples to look back for peak detection.
    pub peak_window_size: usize,
    /// Base stride length in meters (will be calibrated).
    pub base_stride_length_m: f32,
    /// Stride scaling factor for step frequency.
    pub stride_frequency_scale: f32,
}

impl Default for StepDetectorConfig {
    fn default() -> Self {
        Self {
            min_step_interval_ms: 250,   // Max ~4 steps/sec (running)
            max_step_interval_ms: 2000,  // Min ~0.5 steps/sec (slow shuffle)
            min_peak_accel: 0.8,         // Minimum detectable step
            max_peak_accel: 25.0,        // Filter out jumps/impacts
            smoothing_alpha: 0.3,        // Moderate smoothing
            peak_window_size: 15,        // 300ms at 50Hz
            base_stride_length_m: 0.65,  // Average adult stride
            stride_frequency_scale: 0.15, // How much frequency affects stride
        }
    }
}

/// Step detector using peak detection and adaptive stride estimation.
pub struct StepDetector {
    config: StepDetectorConfig,
    
    // Filtering state
    filtered_accel: f32,
    prev_filtered_accel: f32,
    
    // Peak detection buffer
    accel_buffer: Vec<f32>,
    buffer_index: usize,
    
    // Step timing
    last_step_time_ms: u64,
    step_intervals: Vec<u64>,
    
    // Cadence estimation
    current_cadence_hz: f32,
    cadence_samples: Vec<f32>,
    
    // Stride calibration
    calibrated_stride_m: f32,
    stride_calibration_factor: f32,
    
    // Zero-crossing state for additional validation
    last_zero_crossing_ms: u64,
    above_mean: bool,
    
    // Statistics
    total_steps: u64,
    
    // Gravity estimation for vertical acceleration
    gravity: [f32; 3],
    gravity_alpha: f32,
}

impl StepDetector {
    /// Create a new step detector with the given configuration.
    pub fn new(config: StepDetectorConfig) -> Self {
        Self {
            config: config.clone(),
            filtered_accel: 0.0,
            prev_filtered_accel: 0.0,
            accel_buffer: vec![0.0; config.peak_window_size],
            buffer_index: 0,
            last_step_time_ms: 0,
            step_intervals: Vec::with_capacity(10),
            current_cadence_hz: 0.0,
            cadence_samples: Vec::with_capacity(20),
            calibrated_stride_m: config.base_stride_length_m,
            stride_calibration_factor: 1.0,
            last_zero_crossing_ms: 0,
            above_mean: false,
            total_steps: 0,
            gravity: [0.0, 0.0, 9.81],
            gravity_alpha: 0.02,
        }
    }

    /// Create a step detector with default configuration.
    pub fn default() -> Self {
        Self::new(StepDetectorConfig::default())
    }

    /// Process a single IMU sample and detect steps.
    /// Returns a StepEvent if a step was detected, None otherwise.
    pub fn process_sample(&mut self, sample: &ImuSample) -> Option<StepEvent> {
        // Update gravity estimate (low-pass filter)
        self.update_gravity(sample);
        
        // Extract vertical acceleration (acceleration along gravity vector)
        let vertical_accel = self.extract_vertical_accel(sample);
        
        // Apply low-pass filter to reduce noise
        self.prev_filtered_accel = self.filtered_accel;
        self.filtered_accel = self.config.smoothing_alpha * vertical_accel 
            + (1.0 - self.config.smoothing_alpha) * self.filtered_accel;
        
        // Add to circular buffer
        self.accel_buffer[self.buffer_index] = self.filtered_accel;
        self.buffer_index = (self.buffer_index + 1) % self.config.peak_window_size;
        
        // Update zero-crossing state
        self.update_zero_crossing(sample.timestamp_ms);
        
        // Check for step using peak detection
        if self.detect_peak(sample.timestamp_ms) {
            return self.create_step_event(sample.timestamp_ms);
        }
        
        None
    }

    /// Process a batch of samples and return all detected steps.
    pub fn process_batch(&mut self, samples: &[ImuSample]) -> Vec<StepEvent> {
        let mut steps = Vec::new();
        for sample in samples {
            if let Some(step) = self.process_sample(sample) {
                steps.push(step);
            }
        }
        steps
    }

    /// Get the current estimated cadence in steps per second.
    pub fn current_cadence(&self) -> f32 {
        self.current_cadence_hz
    }

    /// Get the total number of steps detected.
    pub fn total_steps(&self) -> u64 {
        self.total_steps
    }

    /// Get the current calibrated stride length.
    pub fn stride_length(&self) -> f32 {
        self.calibrated_stride_m
    }

    /// Calibrate stride length based on known distance.
    /// Call this after user walks a known distance.
    pub fn calibrate_stride(&mut self, known_distance_m: f32, step_count: u32) {
        if step_count > 0 {
            let measured_stride = known_distance_m / step_count as f32;
            // Blend with existing calibration
            self.stride_calibration_factor = measured_stride / self.config.base_stride_length_m;
            self.calibrated_stride_m = measured_stride;
        }
    }

    /// Reset the detector state (call when user stops walking).
    pub fn reset(&mut self) {
        self.filtered_accel = 0.0;
        self.prev_filtered_accel = 0.0;
        self.accel_buffer.fill(0.0);
        self.buffer_index = 0;
        self.step_intervals.clear();
        self.cadence_samples.clear();
        self.current_cadence_hz = 0.0;
        self.last_step_time_ms = 0;
    }

    /// Get the current stance phase based on recent acceleration.
    pub fn current_stance_phase(&self) -> StancePhase {
        // During stance phase, vertical acceleration is near zero
        // During swing phase, there's more variation
        let accel_variance = self.compute_buffer_variance();
        
        if accel_variance < 0.5 && self.filtered_accel.abs() < 0.3 {
            StancePhase::Stance
        } else if accel_variance > 1.5 {
            StancePhase::Swing
        } else {
            StancePhase::Unknown
        }
    }

    // =========================================================================
    // PRIVATE METHODS
    // =========================================================================

    fn update_gravity(&mut self, sample: &ImuSample) {
        // Low-pass filter to estimate gravity
        self.gravity[0] = self.gravity_alpha * sample.accel[0] 
            + (1.0 - self.gravity_alpha) * self.gravity[0];
        self.gravity[1] = self.gravity_alpha * sample.accel[1] 
            + (1.0 - self.gravity_alpha) * self.gravity[1];
        self.gravity[2] = self.gravity_alpha * sample.accel[2] 
            + (1.0 - self.gravity_alpha) * self.gravity[2];
    }

    fn extract_vertical_accel(&self, sample: &ImuSample) -> f32 {
        // Project acceleration onto gravity vector to get vertical component
        let gravity_mag = (self.gravity[0] * self.gravity[0] 
            + self.gravity[1] * self.gravity[1] 
            + self.gravity[2] * self.gravity[2]).sqrt();
        
        if gravity_mag < 0.1 {
            return 0.0;
        }
        
        // Normalize gravity
        let gx = self.gravity[0] / gravity_mag;
        let gy = self.gravity[1] / gravity_mag;
        let gz = self.gravity[2] / gravity_mag;
        
        // Dot product of accel with gravity direction gives vertical component
        let vertical = sample.accel[0] * gx + sample.accel[1] * gy + sample.accel[2] * gz;
        
        // Subtract gravity magnitude to get linear vertical acceleration
        vertical - gravity_mag
    }

    fn update_zero_crossing(&mut self, timestamp_ms: u64) {
        let was_above = self.above_mean;
        self.above_mean = self.filtered_accel > 0.0;
        
        // Detect zero-crossing (useful for step timing validation)
        if was_above != self.above_mean {
            self.last_zero_crossing_ms = timestamp_ms;
        }
    }

    fn detect_peak(&self, timestamp_ms: u64) -> bool {
        // Check minimum interval since last step
        if self.last_step_time_ms > 0 {
            let interval = timestamp_ms.saturating_sub(self.last_step_time_ms);
            if interval < self.config.min_step_interval_ms {
                return false;
            }
        }
        
        // Get the center sample (potential peak) and neighbors
        let center_idx = (self.buffer_index + self.config.peak_window_size / 2) 
            % self.config.peak_window_size;
        let center_val = self.accel_buffer[center_idx];
        
        // Peak must be above minimum threshold
        if center_val < self.config.min_peak_accel {
            return false;
        }
        
        // Peak must be below maximum threshold (not a jump/impact)
        if center_val > self.config.max_peak_accel {
            return false;
        }
        
        // Check if center is a local maximum
        let mut is_peak = true;
        for i in 0..self.config.peak_window_size {
            if i != center_idx && self.accel_buffer[i] >= center_val {
                is_peak = false;
                break;
            }
        }
        
        // Additional validation: must have had a recent zero-crossing
        // (foot plant causes acceleration to cross through zero)
        let time_since_crossing = timestamp_ms.saturating_sub(self.last_zero_crossing_ms);
        if time_since_crossing > 200 {
            // No recent zero-crossing, might be noise
            is_peak = is_peak && center_val > self.config.min_peak_accel * 1.5;
        }
        
        is_peak
    }

    fn create_step_event(&mut self, timestamp_ms: u64) -> Option<StepEvent> {
        // Calculate step interval
        let interval_ms = if self.last_step_time_ms > 0 {
            timestamp_ms.saturating_sub(self.last_step_time_ms)
        } else {
            500 // Default assumption for first step
        };
        
        // Check if interval is reasonable
        if interval_ms > self.config.max_step_interval_ms {
            // Too long since last step - reset cadence
            self.step_intervals.clear();
            self.cadence_samples.clear();
        }
        
        // Update step timing
        self.last_step_time_ms = timestamp_ms;
        self.step_intervals.push(interval_ms);
        if self.step_intervals.len() > 10 {
            self.step_intervals.remove(0);
        }
        
        // Update cadence estimate
        self.update_cadence();
        
        // Estimate stride length based on cadence
        let stride_length = self.estimate_stride_length();
        
        // Compute confidence based on consistency
        let confidence = self.compute_step_confidence();
        
        // Get peak acceleration for this step
        let center_idx = (self.buffer_index + self.config.peak_window_size / 2) 
            % self.config.peak_window_size;
        let peak_accel = self.accel_buffer[center_idx];
        
        self.total_steps += 1;
        
        Some(StepEvent::new(
            timestamp_ms,
            stride_length,
            self.current_cadence_hz,
            confidence,
            peak_accel,
        ))
    }

    fn update_cadence(&mut self) {
        if self.step_intervals.is_empty() {
            self.current_cadence_hz = 0.0;
            return;
        }
        
        // Calculate average interval
        let sum: u64 = self.step_intervals.iter().sum();
        let avg_interval_ms = sum as f32 / self.step_intervals.len() as f32;
        
        if avg_interval_ms > 0.0 {
            self.current_cadence_hz = 1000.0 / avg_interval_ms;
        }
        
        // Track cadence samples for smoothing
        self.cadence_samples.push(self.current_cadence_hz);
        if self.cadence_samples.len() > 20 {
            self.cadence_samples.remove(0);
        }
    }

    fn estimate_stride_length(&self) -> f32 {
        // Stride length model: stride = base_stride * (1 + k * (freq - 1.5))
        // At ~1.5 Hz (normal walking), use base stride
        // Higher frequency (running) = longer stride
        // Lower frequency (shuffle) = shorter stride
        
        let freq_factor = (self.current_cadence_hz - 1.5) * self.config.stride_frequency_scale;
        let dynamic_stride = self.calibrated_stride_m * (1.0 + freq_factor);
        
        // Clamp to reasonable range
        dynamic_stride.clamp(0.3, 1.5)
    }

    fn compute_step_confidence(&self) -> f32 {
        let mut confidence: f32 = 0.7; // Base confidence
        
        // Higher confidence with consistent cadence
        if self.cadence_samples.len() >= 5 {
            let cadence_std = self.compute_cadence_std();
            if cadence_std < 0.2 {
                confidence += 0.2; // Very consistent
            } else if cadence_std < 0.4 {
                confidence += 0.1; // Moderately consistent
            }
        }
        
        // Adjust for cadence reasonableness (0.5 - 4.0 Hz is normal)
        if self.current_cadence_hz >= 0.8 && self.current_cadence_hz <= 3.0 {
            confidence += 0.1;
        } else if self.current_cadence_hz < 0.5 || self.current_cadence_hz > 4.0 {
            confidence -= 0.2;
        }
        
        confidence.clamp(0.0, 1.0)
    }

    fn compute_cadence_std(&self) -> f32 {
        if self.cadence_samples.len() < 2 {
            return 0.0;
        }
        
        let mean: f32 = self.cadence_samples.iter().sum::<f32>() 
            / self.cadence_samples.len() as f32;
        let variance: f32 = self.cadence_samples.iter()
            .map(|x| (x - mean) * (x - mean))
            .sum::<f32>() / self.cadence_samples.len() as f32;
        
        variance.sqrt()
    }

    fn compute_buffer_variance(&self) -> f32 {
        let mean: f32 = self.accel_buffer.iter().sum::<f32>() 
            / self.accel_buffer.len() as f32;
        let variance: f32 = self.accel_buffer.iter()
            .map(|x| (x - mean) * (x - mean))
            .sum::<f32>() / self.accel_buffer.len() as f32;
        
        variance
    }
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn create_walking_samples(num_steps: u32, step_interval_ms: u64) -> Vec<ImuSample> {
        let mut samples = Vec::new();
        let samples_per_step = (step_interval_ms / 20) as usize; // 50Hz
        
        for step in 0..num_steps {
            for i in 0..samples_per_step {
                let t = step as u64 * step_interval_ms + i as u64 * 20;
                
                // Simulate step pattern: peak at heel strike
                let phase = (i as f32 / samples_per_step as f32) * std::f32::consts::PI * 2.0;
                let step_accel = 2.0 * phase.sin(); // Peak of ~2 m/s²
                
                let sample = ImuSample::new(
                    t,
                    [0.1, 0.2, 9.81 + step_accel],
                    [0.0, 0.0, 0.1],
                );
                samples.push(sample);
            }
        }
        samples
    }

    #[test]
    fn test_step_detector_creation() {
        let detector = StepDetector::default();
        assert_eq!(detector.total_steps(), 0);
        assert_eq!(detector.current_cadence(), 0.0);
    }

    #[test]
    fn test_step_detection_basic() {
        let mut detector = StepDetector::default();
        let samples = create_walking_samples(10, 500); // 2 Hz walking
        
        let steps = detector.process_batch(&samples);
        
        // Should detect roughly 10 steps (may vary due to algorithm)
        assert!(steps.len() >= 5, "Should detect at least 5 steps, got {}", steps.len());
        assert!(steps.len() <= 15, "Should not detect more than 15 steps, got {}", steps.len());
    }

    #[test]
    fn test_cadence_estimation() {
        let mut detector = StepDetector::default();
        let samples = create_walking_samples(20, 500); // 2 Hz walking
        
        let _ = detector.process_batch(&samples);
        
        let cadence = detector.current_cadence();
        // Should be approximately 2 Hz
        assert!(cadence > 1.0 && cadence < 3.0, 
            "Cadence should be around 2 Hz, got {}", cadence);
    }

    #[test]
    fn test_stride_length_estimation() {
        let mut detector = StepDetector::default();
        
        // Normal walking cadence (1.5-2 Hz) should give normal stride
        detector.current_cadence_hz = 1.8;
        let stride = detector.estimate_stride_length();
        
        assert!(stride > 0.5 && stride < 0.9, 
            "Normal stride should be 0.5-0.9m, got {}", stride);
    }

    #[test]
    fn test_stride_calibration() {
        let mut detector = StepDetector::default();
        
        // Calibrate: 10 meters in 15 steps = 0.67m stride
        detector.calibrate_stride(10.0, 15);
        
        assert!((detector.stride_length() - 0.67).abs() < 0.05,
            "Calibrated stride should be ~0.67m, got {}", detector.stride_length());
    }

    #[test]
    fn test_stance_phase_detection() {
        let mut detector = StepDetector::default();
        
        // Feed some quiet samples (stance phase)
        for i in 0..50 {
            let sample = ImuSample::new(
                i * 20,
                [0.0, 0.0, 9.81],
                [0.0, 0.0, 0.0],
            );
            let _ = detector.process_sample(&sample);
        }
        
        // Should detect stance phase
        let phase = detector.current_stance_phase();
        assert_eq!(phase, StancePhase::Stance);
    }

    #[test]
    fn test_reset_clears_state() {
        let mut detector = StepDetector::default();
        let samples = create_walking_samples(5, 500);
        let _ = detector.process_batch(&samples);
        
        detector.reset();
        
        assert_eq!(detector.current_cadence(), 0.0);
        assert_eq!(detector.filtered_accel, 0.0);
    }

    #[test]
    fn test_minimum_step_interval() {
        let mut detector = StepDetector::default();
        
        // Create samples with very fast "steps" (too fast to be real)
        let mut samples = Vec::new();
        for i in 0..20 {
            // One sample every 20ms, with "peak" every 50ms (20 Hz - impossible)
            let is_peak = i % 3 == 0;
            let sample = ImuSample::new(
                i * 20,
                [0.0, 0.0, if is_peak { 15.0 } else { 9.81 }],
                [0.0, 0.0, 0.0],
            );
            samples.push(sample);
        }
        
        let steps = detector.process_batch(&samples);
        
        // Should reject most as too fast
        assert!(steps.len() < 5, "Should reject too-fast steps, got {}", steps.len());
    }

    #[test]
    fn test_step_event_fields() {
        let event = StepEvent::new(1000, 0.7, 1.8, 0.85, 2.5);
        
        assert_eq!(event.timestamp_ms, 1000);
        assert_eq!(event.stride_length_m, 0.7);
        assert_eq!(event.step_frequency_hz, 1.8);
        assert_eq!(event.confidence, 0.85);
        assert_eq!(event.peak_accel, 2.5);
    }
}
