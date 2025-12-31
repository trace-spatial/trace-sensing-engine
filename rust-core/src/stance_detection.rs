//! Stance Detection Module.
//!
//! Implements robust stance/swing phase detection for gait analysis.
//! Uses a combination of acceleration magnitude, gyroscope activity,
//! and temporal patterns to identify when the foot is in contact
//! with the ground (stance) vs moving (swing).
//!
//! Stance detection is critical for:
//! - Zero Velocity Updates (ZUPT) in PDR
//! - Step segmentation
//! - Gait pattern analysis
//!
//! The detector uses a sliding window approach with multiple features
//! to achieve robust detection across different walking styles.

use crate::types::{ImuSample, StancePhase};

/// Configuration for stance detection.
#[derive(Debug, Clone)]
pub struct StanceDetectorConfig {
    /// Window size in samples for stance detection.
    pub window_size: usize,
    /// Acceleration magnitude threshold below which stance is likely (m/s²).
    pub accel_threshold: f32,
    /// Gyroscope magnitude threshold below which stance is likely (rad/s).
    pub gyro_threshold: f32,
    /// Acceleration variance threshold for stance detection.
    pub accel_variance_threshold: f32,
    /// Minimum consecutive samples in stance to confirm stance phase.
    pub min_stance_samples: usize,
    /// Minimum consecutive samples in swing to confirm swing phase.
    pub min_swing_samples: usize,
    /// Gravity magnitude for reference (m/s²).
    pub gravity_magnitude: f32,
}

impl Default for StanceDetectorConfig {
    fn default() -> Self {
        Self {
            window_size: 10,              // 200ms at 50Hz
            accel_threshold: 0.5,          // m/s² deviation from gravity
            gyro_threshold: 0.3,           // rad/s
            accel_variance_threshold: 0.3, // Low variance during stance
            min_stance_samples: 3,         // ~60ms at 50Hz
            min_swing_samples: 3,          // ~60ms at 50Hz
            gravity_magnitude: 9.81,
        }
    }
}

/// Feature vector for stance detection.
#[derive(Debug, Clone, Copy, Default)]
struct StanceFeatures {
    /// Acceleration magnitude deviation from gravity.
    accel_deviation: f32,
    /// Gyroscope magnitude.
    gyro_magnitude: f32,
    /// Acceleration variance over window.
    accel_variance: f32,
    /// Gyroscope variance over window.
    gyro_variance: f32,
    /// Jerk (derivative of acceleration) magnitude.
    jerk_magnitude: f32,
}

/// Stance detector using multi-feature analysis.
pub struct StanceDetector {
    config: StanceDetectorConfig,
    
    // Circular buffer for acceleration magnitudes
    accel_buffer: Vec<f32>,
    gyro_buffer: Vec<f32>,
    buffer_index: usize,
    buffer_filled: bool,
    
    // Previous values for jerk calculation
    prev_accel: [f32; 3],
    prev_timestamp_ms: u64,
    
    // State tracking
    current_phase: StancePhase,
    phase_sample_count: usize,
    
    // Gravity estimation
    gravity: [f32; 3],
    gravity_alpha: f32,
    
    // Statistics
    stance_count: u64,
    swing_count: u64,
}

impl StanceDetector {
    /// Create a new stance detector with the given configuration.
    pub fn new(config: StanceDetectorConfig) -> Self {
        let window_size = config.window_size;
        Self {
            config,
            accel_buffer: vec![0.0; window_size],
            gyro_buffer: vec![0.0; window_size],
            buffer_index: 0,
            buffer_filled: false,
            prev_accel: [0.0, 0.0, 9.81],
            prev_timestamp_ms: 0,
            current_phase: StancePhase::Unknown,
            phase_sample_count: 0,
            gravity: [0.0, 0.0, 9.81],
            gravity_alpha: 0.02,
            stance_count: 0,
            swing_count: 0,
        }
    }

    /// Create a stance detector with default configuration.
    pub fn default_detector() -> Self {
        Self::new(StanceDetectorConfig::default())
    }

    /// Process a single IMU sample and determine stance phase.
    pub fn process_sample(&mut self, sample: &ImuSample) -> StancePhase {
        // Update gravity estimate
        self.update_gravity(sample);
        
        // Compute features
        let features = self.compute_features(sample);
        
        // Update buffers
        self.update_buffers(&features);
        
        // Store for next iteration
        self.prev_accel = sample.accel;
        self.prev_timestamp_ms = sample.timestamp_ms;
        
        // Classify stance phase
        let raw_phase = self.classify_phase(&features);
        
        // Apply hysteresis
        self.apply_hysteresis(raw_phase)
    }

    /// Get the current stance phase.
    pub fn current_phase(&self) -> StancePhase {
        self.current_phase
    }

    /// Get the probability of being in stance phase [0.0, 1.0].
    pub fn stance_probability(&self) -> f32 {
        if !self.buffer_filled {
            return 0.5; // Unknown
        }
        
        // Compute features from current buffer
        let avg_accel = self.accel_buffer.iter().sum::<f32>() / self.accel_buffer.len() as f32;
        let avg_gyro = self.gyro_buffer.iter().sum::<f32>() / self.gyro_buffer.len() as f32;
        
        // Score based on thresholds
        let accel_score = 1.0 - (avg_accel / self.config.accel_threshold).min(1.0);
        let gyro_score = 1.0 - (avg_gyro / self.config.gyro_threshold).min(1.0);
        
        // Combine scores
        (accel_score * 0.6 + gyro_score * 0.4).clamp(0.0, 1.0)
    }

    /// Check if ZUPT (Zero Velocity Update) should be applied.
    /// Returns true if we're confident the foot is stationary.
    pub fn should_apply_zupt(&self) -> bool {
        self.current_phase == StancePhase::Stance 
            && self.phase_sample_count >= self.config.min_stance_samples
            && self.stance_probability() > 0.7
    }

    /// Get the confidence in the current phase detection [0.0, 1.0].
    pub fn phase_confidence(&self) -> f32 {
        if !self.buffer_filled {
            return 0.3; // Low confidence when warming up
        }
        
        // Higher confidence with more samples in current phase
        let sample_factor = (self.phase_sample_count as f32 / 10.0).min(1.0);
        
        // Compute variance for confidence
        let accel_variance = self.compute_variance(&self.accel_buffer);
        let variance_factor = 1.0 - (accel_variance / 2.0).min(1.0);
        
        (0.5 + sample_factor * 0.3 + variance_factor * 0.2).clamp(0.0, 1.0)
    }

    /// Reset the detector state.
    pub fn reset(&mut self) {
        self.accel_buffer.fill(0.0);
        self.gyro_buffer.fill(0.0);
        self.buffer_index = 0;
        self.buffer_filled = false;
        self.current_phase = StancePhase::Unknown;
        self.phase_sample_count = 0;
        self.prev_accel = [0.0, 0.0, 9.81];
        self.prev_timestamp_ms = 0;
    }

    /// Get stance statistics (stance_count, swing_count).
    pub fn statistics(&self) -> (u64, u64) {
        (self.stance_count, self.swing_count)
    }

    // =========================================================================
    // PRIVATE METHODS
    // =========================================================================

    fn update_gravity(&mut self, sample: &ImuSample) {
        self.gravity[0] = self.gravity_alpha * sample.accel[0] 
            + (1.0 - self.gravity_alpha) * self.gravity[0];
        self.gravity[1] = self.gravity_alpha * sample.accel[1] 
            + (1.0 - self.gravity_alpha) * self.gravity[1];
        self.gravity[2] = self.gravity_alpha * sample.accel[2] 
            + (1.0 - self.gravity_alpha) * self.gravity[2];
    }

    fn compute_features(&self, sample: &ImuSample) -> StanceFeatures {
        // Acceleration magnitude
        let accel_mag = (sample.accel[0] * sample.accel[0]
            + sample.accel[1] * sample.accel[1]
            + sample.accel[2] * sample.accel[2]).sqrt();
        
        // Deviation from gravity
        let accel_deviation = (accel_mag - self.config.gravity_magnitude).abs();
        
        // Gyroscope magnitude
        let gyro_magnitude = (sample.gyro[0] * sample.gyro[0]
            + sample.gyro[1] * sample.gyro[1]
            + sample.gyro[2] * sample.gyro[2]).sqrt();
        
        // Jerk (change in acceleration)
        let dt = if self.prev_timestamp_ms > 0 {
            (sample.timestamp_ms - self.prev_timestamp_ms) as f32 / 1000.0
        } else {
            0.02 // Assume 50Hz
        };
        
        let jerk_magnitude = if dt > 0.0 {
            let jx = (sample.accel[0] - self.prev_accel[0]) / dt;
            let jy = (sample.accel[1] - self.prev_accel[1]) / dt;
            let jz = (sample.accel[2] - self.prev_accel[2]) / dt;
            (jx * jx + jy * jy + jz * jz).sqrt()
        } else {
            0.0
        };
        
        StanceFeatures {
            accel_deviation,
            gyro_magnitude,
            accel_variance: 0.0, // Will be computed from buffer
            gyro_variance: 0.0,
            jerk_magnitude,
        }
    }

    fn update_buffers(&mut self, features: &StanceFeatures) {
        self.accel_buffer[self.buffer_index] = features.accel_deviation;
        self.gyro_buffer[self.buffer_index] = features.gyro_magnitude;
        
        self.buffer_index = (self.buffer_index + 1) % self.config.window_size;
        if self.buffer_index == 0 {
            self.buffer_filled = true;
        }
    }

    fn classify_phase(&self, features: &StanceFeatures) -> StancePhase {
        if !self.buffer_filled {
            return StancePhase::Unknown;
        }
        
        // Compute windowed statistics
        let avg_accel = self.accel_buffer.iter().sum::<f32>() / self.accel_buffer.len() as f32;
        let avg_gyro = self.gyro_buffer.iter().sum::<f32>() / self.gyro_buffer.len() as f32;
        let accel_variance = self.compute_variance(&self.accel_buffer);
        
        // Multi-feature classification
        let is_low_accel = avg_accel < self.config.accel_threshold;
        let is_low_gyro = avg_gyro < self.config.gyro_threshold;
        let is_low_variance = accel_variance < self.config.accel_variance_threshold;
        let is_low_jerk = features.jerk_magnitude < 10.0; // m/s³
        
        // Stance: low acceleration deviation, low gyro, low variance
        let stance_votes = [is_low_accel, is_low_gyro, is_low_variance, is_low_jerk]
            .iter()
            .filter(|&&x| x)
            .count();
        
        if stance_votes >= 3 {
            StancePhase::Stance
        } else if stance_votes <= 1 {
            StancePhase::Swing
        } else {
            StancePhase::Unknown
        }
    }

    fn apply_hysteresis(&mut self, raw_phase: StancePhase) -> StancePhase {
        if raw_phase == self.current_phase {
            self.phase_sample_count += 1;
            return self.current_phase;
        }
        
        // Phase change detected - check if we have enough samples
        let min_samples = match raw_phase {
            StancePhase::Stance => self.config.min_stance_samples,
            StancePhase::Swing => self.config.min_swing_samples,
            StancePhase::Unknown => 1,
        };
        
        self.phase_sample_count += 1;
        
        if self.phase_sample_count >= min_samples || self.current_phase == StancePhase::Unknown {
            // Confirm phase change
            let old_phase = self.current_phase;
            self.current_phase = raw_phase;
            self.phase_sample_count = 1;
            
            // Update statistics
            match old_phase {
                StancePhase::Stance => self.stance_count += 1,
                StancePhase::Swing => self.swing_count += 1,
                StancePhase::Unknown => {}
            }
        }
        
        self.current_phase
    }

    fn compute_variance(&self, buffer: &[f32]) -> f32 {
        let n = buffer.len() as f32;
        if n < 2.0 {
            return 0.0;
        }
        
        let mean = buffer.iter().sum::<f32>() / n;
        buffer.iter()
            .map(|x| (x - mean) * (x - mean))
            .sum::<f32>() / n
    }
}

/// Advanced stance detector using multiple detection methods.
/// Combines shoe-mounted IMU patterns with phone-in-pocket patterns.
pub struct AdaptiveStanceDetector {
    /// Primary detector using acceleration patterns.
    accel_detector: StanceDetector,
    
    /// Confidence weighting for combining detectors.
    accel_weight: f32,
    
    // Shoe-mounted specific patterns (if phone is on foot)
    shoe_mode: bool,
    
    // Pocket-specific patterns (if phone is in pocket)
    pocket_mode: bool,
}

impl AdaptiveStanceDetector {
    pub fn new() -> Self {
        Self {
            accel_detector: StanceDetector::default_detector(),
            accel_weight: 1.0,
            shoe_mode: false,
            pocket_mode: true, // Default assumption
        }
    }

    /// Process sample and return stance phase with confidence.
    pub fn process_sample(&mut self, sample: &ImuSample) -> (StancePhase, f32) {
        let phase = self.accel_detector.process_sample(sample);
        let confidence = self.accel_detector.phase_confidence();
        
        (phase, confidence)
    }

    /// Check if ZUPT should be applied.
    pub fn should_apply_zupt(&self) -> bool {
        self.accel_detector.should_apply_zupt()
    }

    /// Set detection mode based on phone placement.
    pub fn set_mode(&mut self, shoe_mode: bool, pocket_mode: bool) {
        self.shoe_mode = shoe_mode;
        self.pocket_mode = pocket_mode;
        
        // Adjust thresholds based on mode
        if shoe_mode {
            // Shoe-mounted: can use stricter thresholds
            self.accel_detector.config.accel_threshold = 0.3;
            self.accel_detector.config.gyro_threshold = 0.2;
        } else if pocket_mode {
            // Pocket: more relaxed thresholds
            self.accel_detector.config.accel_threshold = 0.6;
            self.accel_detector.config.gyro_threshold = 0.4;
        }
    }

    pub fn reset(&mut self) {
        self.accel_detector.reset();
    }
}

impl Default for AdaptiveStanceDetector {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn create_stance_samples(count: usize) -> Vec<ImuSample> {
        (0..count).map(|i| {
            ImuSample::new(
                i as u64 * 20,
                [0.01, 0.02, 9.81], // Very still
                [0.01, 0.01, 0.01], // Very low rotation
            )
        }).collect()
    }

    fn create_swing_samples(count: usize) -> Vec<ImuSample> {
        (0..count).map(|i| {
            let phase = (i as f32 * 0.5).sin();
            ImuSample::new(
                i as u64 * 20,
                [1.0 + phase, 0.5, 9.81 + 2.0 * phase], // Active motion
                [0.5, 0.3, 0.2 + phase * 0.3],          // Rotation
            )
        }).collect()
    }

    #[test]
    fn test_stance_detector_creation() {
        let detector = StanceDetector::default_detector();
        assert_eq!(detector.current_phase(), StancePhase::Unknown);
    }

    #[test]
    fn test_stance_detection() {
        let mut detector = StanceDetector::default_detector();
        let samples = create_stance_samples(30);
        
        let mut final_phase = StancePhase::Unknown;
        for sample in &samples {
            final_phase = detector.process_sample(sample);
        }
        
        assert_eq!(final_phase, StancePhase::Stance, 
            "Should detect stance phase for still samples");
    }

    #[test]
    fn test_swing_detection() {
        let mut detector = StanceDetector::default_detector();
        let samples = create_swing_samples(30);
        
        let mut final_phase = StancePhase::Unknown;
        for sample in &samples {
            final_phase = detector.process_sample(sample);
        }
        
        assert_eq!(final_phase, StancePhase::Swing,
            "Should detect swing phase for moving samples");
    }

    #[test]
    fn test_stance_probability() {
        let mut detector = StanceDetector::default_detector();
        
        // Process stance samples
        let stance_samples = create_stance_samples(20);
        for sample in &stance_samples {
            let _ = detector.process_sample(sample);
        }
        
        let prob = detector.stance_probability();
        assert!(prob > 0.6, "Stance probability should be high, got {}", prob);
    }

    #[test]
    fn test_zupt_application() {
        let mut detector = StanceDetector::default_detector();
        
        // Process enough stance samples
        let samples = create_stance_samples(30);
        for sample in &samples {
            let _ = detector.process_sample(sample);
        }
        
        assert!(detector.should_apply_zupt(),
            "Should apply ZUPT during confirmed stance");
    }

    #[test]
    fn test_phase_transition() {
        let mut detector = StanceDetector::default_detector();
        
        // Start with stance (timestamps 0..400ms)
        let stance_samples = create_stance_samples(20);
        for sample in &stance_samples {
            let _ = detector.process_sample(sample);
        }
        // After 20 samples at 20ms each, prev_timestamp_ms = 380
        
        assert_eq!(detector.current_phase(), StancePhase::Stance);
        
        // Transition to swing - use continuing timestamps
        let swing_samples: Vec<ImuSample> = (0..20).map(|i| {
            let phase = (i as f32 * 0.5).sin();
            ImuSample::new(
                400 + (i as u64 * 20), // Continue from 400ms
                [1.0 + phase, 0.5, 9.81 + 2.0 * phase],
                [0.5, 0.3, 0.2 + phase * 0.3],
            )
        }).collect();
        
        for sample in &swing_samples {
            let _ = detector.process_sample(sample);
        }
        assert_eq!(detector.current_phase(), StancePhase::Swing);
    }

    #[test]
    fn test_reset() {
        let mut detector = StanceDetector::default_detector();
        
        // Process some samples
        let samples = create_stance_samples(20);
        for sample in &samples {
            let _ = detector.process_sample(sample);
        }
        
        detector.reset();
        
        assert_eq!(detector.current_phase(), StancePhase::Unknown);
        assert!(!detector.buffer_filled);
    }

    #[test]
    fn test_adaptive_detector() {
        let mut detector = AdaptiveStanceDetector::new();
        
        let samples = create_stance_samples(30);
        let mut final_confidence = 0.0;
        
        for sample in &samples {
            let (_, conf) = detector.process_sample(sample);
            final_confidence = conf;
        }
        
        assert!(final_confidence > 0.5, 
            "Confidence should be reasonable, got {}", final_confidence);
    }
}
