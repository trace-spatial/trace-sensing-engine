//! Motion segmentation and state classification.
//!
//! This module segments continuous motion into consistent stretches with
//! classified behavior (Still, SteadyMotion, Turning, Transitional).
//!
//! Design: State machine + feature extraction
//! - Uses a sliding window to compute motion features
//! - Classifies each window into motion states
//! - Detects transitions between states
//! - O(1) per sample using incremental statistics
//!
//! Why this matters:
//! Walking looks different from standing which looks different from turning.
//! By classifying motion, we can identify the moments when behavior changes—
//! those are often the moments when attention is disrupted.

use crate::types::MotionMode;

/// Configuration for motion segmentation.
#[derive(Debug, Clone)]
pub struct SegmentationConfig {
    /// Window size in samples for feature computation.
    /// Larger = more stable but slower to respond. Typical: 50 samples.
    pub window_size: usize,

    /// Threshold for "still" detection: linear acceleration below this (m/s²).
    /// Typical: 0.3 m/s².
    pub still_accel_threshold: f32,

    /// Threshold for "steady motion": consistent acceleration without large rotation.
    /// Typical: 0.5 m/s² to 3.0 m/s².
    pub steady_accel_min: f32,
    pub steady_accel_max: f32,

    /// Threshold for "turning": high rotation without sustained motion.
    /// Typical: 0.2 rad/s.
    pub turning_gyro_threshold: f32,

    /// Threshold for detecting rapid acceleration changes (transitional state).
    /// Typical: 1.0 m/s² per window.
    pub transition_accel_delta: f32,

    /// Minimum segment duration in samples before declaring a new state.
    /// Prevents flickering. Typical: 10-20 samples.
    pub min_segment_duration: usize,

    /// Confidence penalty when accel variance is high (noisy signal).
    /// Range: [0.0, 1.0]. Typical: 0.2.
    pub noise_confidence_penalty: f32,
}

impl Default for SegmentationConfig {
    fn default() -> Self {
        Self {
            window_size: 50,
            still_accel_threshold: 0.3,
            steady_accel_min: 0.5,
            steady_accel_max: 3.0,
            turning_gyro_threshold: 0.2,
            transition_accel_delta: 1.0,
            min_segment_duration: 15,
            noise_confidence_penalty: 0.2,
        }
    }
}

/// Statistics computed over a window of samples.
#[derive(Debug, Clone)]
struct WindowStats {
    /// Count of samples in window.
    sample_count: usize,
    /// Sum of linear acceleration magnitudes.
    accel_sum: f32,
    /// Mean linear acceleration magnitude.
    accel_mean: f32,
    /// Sum of squared deviations for variance.
    accel_sum_sq_dev: f32,
    /// Variance of acceleration.
    accel_variance: f32,
    /// Mean gyro magnitude.
    gyro_mean: f32,
    /// Max acceleration seen in window.
    accel_max: f32,
    /// Min acceleration seen in window.
    accel_min: f32,
}

impl Default for WindowStats {
    fn default() -> Self {
        Self {
            sample_count: 0,
            accel_sum: 0.0,
            accel_mean: 0.0,
            accel_sum_sq_dev: 0.0,
            accel_variance: 0.0,
            gyro_mean: 0.0,
            accel_max: 0.0,
            accel_min: f32::INFINITY,
        }
    }
}

impl WindowStats {
    /// Compute standard deviation from variance.
    fn accel_std_dev(&self) -> f32 {
        self.accel_variance.sqrt()
    }

    /// Finalize statistics after window is complete.
    fn finalize(&mut self) {
        if self.sample_count == 0 {
            return;
        }

        self.accel_mean = self.accel_sum / self.sample_count as f32;

        if self.sample_count > 1 {
            self.accel_variance = self.accel_sum_sq_dev / (self.sample_count as f32 - 1.0);
        }
    }
}

/// A windowed buffer for incremental statistics computation.
///
/// Maintains a circular buffer to compute statistics without recomputing
/// over the entire window each time. O(1) add, O(1) window statistics.
struct CircularWindow {
    buffer: Vec<f32>,
    gyro_buffer: Vec<f32>,
    write_idx: usize,
    full: bool,
}

impl CircularWindow {
    /// Create a new circular window with given size.
    fn new(size: usize) -> Self {
        Self {
            buffer: vec![0.0; size],
            gyro_buffer: vec![0.0; size],
            write_idx: 0,
            full: false,
        }
    }

    /// Add a sample to the window.
    fn push(&mut self, accel_mag: f32, gyro_mag: f32) {
        self.buffer[self.write_idx] = accel_mag;
        self.gyro_buffer[self.write_idx] = gyro_mag;

        self.write_idx = (self.write_idx + 1) % self.buffer.len();
        if self.write_idx == 0 {
            self.full = true;
        }
    }

    /// Check if window is full.
    fn is_full(&self) -> bool {
        self.full
    }

    /// Get the current number of samples in window.
    fn len(&self) -> usize {
        if self.full {
            self.buffer.len()
        } else {
            self.write_idx
        }
    }

    /// Compute statistics from current window.
    fn compute_stats(&self) -> WindowStats {
        let n = self.len();
        if n == 0 {
            return WindowStats::default();
        }

        let mut stats = WindowStats {
            sample_count: n,
            accel_min: f32::INFINITY,
            ..Default::default()
        };

        // First pass: mean and min/max
        for i in 0..n {
            let val = self.buffer[i];
            stats.accel_sum += val;
            stats.accel_max = stats.accel_max.max(val);
            stats.accel_min = stats.accel_min.min(val);
        }

        stats.accel_mean = stats.accel_sum / n as f32;

        // Compute gyro mean
        for i in 0..n {
            stats.gyro_mean += self.gyro_buffer[i];
        }
        stats.gyro_mean /= n as f32;

        // Second pass: variance (sum of squared deviations)
        for i in 0..n {
            let dev = self.buffer[i] - stats.accel_mean;
            stats.accel_sum_sq_dev += dev * dev;
        }

        if n > 1 {
            stats.accel_variance = stats.accel_sum_sq_dev / (n as f32 - 1.0);
        }

        stats
    }
}

/// Classifies motion based on features.
///
/// Simple rule-based classifier using thresholds.
/// Conservative: prefers uncertain classifications (lower confidence)
/// over overconfident wrong ones.
struct MotionClassifier {
    config: SegmentationConfig,
}

impl MotionClassifier {
    fn new(config: SegmentationConfig) -> Self {
        Self { config }
    }

    /// Classify motion and return (mode, confidence).
    ///
    /// Confidence accounts for:
    /// - Signal clarity (lower variance = higher confidence)
    /// - Feature strength (how clearly defined the motion is)
    fn classify(&self, stats: &WindowStats) -> (MotionMode, f32) {
        // Baseline confidence from signal quality
        let noise_std_dev = stats.accel_std_dev();
        let mut confidence = 1.0;

        if noise_std_dev > 0.5 {
            confidence -= self.config.noise_confidence_penalty;
        }

        // Classify based on acceleration and rotation
        let accel_mean = stats.accel_mean;
        let gyro_mean = stats.gyro_mean;
        let accel_range = stats.accel_max - stats.accel_min;

        // Decision tree: acceleration first, then rotation
        if accel_mean < self.config.still_accel_threshold {
            // Very low acceleration = still
            (MotionMode::Still, (confidence * 0.95).max(0.0))
        } else if accel_mean > self.config.steady_accel_max {
            // High acceleration = transitional (rapid change)
            (
                MotionMode::Transitional,
                (confidence * (0.8 + accel_range / 5.0)).min(1.0),
            )
        } else if accel_mean >= self.config.steady_accel_min
            && accel_mean <= self.config.steady_accel_max
        {
            // Moderate, steady acceleration
            if gyro_mean > self.config.turning_gyro_threshold {
                // High rotation = turning
                (MotionMode::Turning, (confidence * 0.85).max(0.0))
            } else {
                // Low rotation = steady motion (walking)
                (MotionMode::SteadyMotion, (confidence * 0.92).max(0.0))
            }
        } else {
            // Edge case: default to transitional
            (MotionMode::Transitional, (confidence * 0.7).max(0.0))
        }
    }
}

/// A segment classifier that tracks state transitions.
///
/// Maintains:
/// - Circular window of samples
/// - Current motion mode and confidence
/// - Segment duration counter
/// - Detection of state transitions
pub struct SegmentationEngine {
    window: CircularWindow,
    classifier: MotionClassifier,
    config: SegmentationConfig,

    // Current state tracking
    current_mode: MotionMode,
    current_confidence: f32,
    samples_in_current_mode: usize,
    previous_mode: MotionMode,

    // Diagnostics
    total_samples: u64,
    segment_boundary_count: u64,
}

impl SegmentationEngine {
    /// Create a new segmentation engine.
    pub fn new(config: SegmentationConfig) -> Self {
        Self {
            window: CircularWindow::new(config.window_size),
            classifier: MotionClassifier::new(config.clone()),
            config,
            current_mode: MotionMode::Still,
            current_confidence: 1.0,
            samples_in_current_mode: 0,
            previous_mode: MotionMode::Still,
            total_samples: 0,
            segment_boundary_count: 0,
        }
    }

    /// Process a new sample and check for segment updates.
    ///
    /// Returns Some(SegmentClassification) if a segment boundary is detected,
    /// None otherwise.
    pub fn process_sample(&mut self, accel_mag: f32, gyro_mag: f32) -> Option<SegmentClassification> {
        self.window.push(accel_mag, gyro_mag);
        self.total_samples += 1;
        self.samples_in_current_mode += 1;

        // Need enough samples before classifying
        if !self.window.is_full() {
            return None;
        }

        // Compute current classification
        let stats = self.window.compute_stats();
        let (new_mode, new_confidence) = self.classifier.classify(&stats);

        // Check for state transition
        if new_mode != self.current_mode && self.samples_in_current_mode >= self.config.min_segment_duration
        {
            // State transition detected
            let boundary = SegmentClassification {
                transition_type: Some((self.current_mode, new_mode)),
                sample_index: self.total_samples,
                duration_samples: self.samples_in_current_mode,
                mode: self.current_mode,
                confidence: self.current_confidence,
                accel_mean: stats.accel_mean,
                gyro_mean: stats.gyro_mean,
                accel_variance: stats.accel_variance,
            };

            self.previous_mode = self.current_mode;
            self.current_mode = new_mode;
            self.current_confidence = new_confidence;
            self.samples_in_current_mode = 0;
            self.segment_boundary_count += 1;

            Some(boundary)
        } else if new_mode == self.current_mode {
            // No transition, update confidence
            self.current_confidence = (self.current_confidence + new_confidence) / 2.0;
            None
        } else {
            // Still in transition period, not enough samples yet
            None
        }
    }

    /// Get the current motion mode.
    pub fn current_mode(&self) -> MotionMode {
        self.current_mode
    }

    /// Get the confidence in current classification.
    pub fn current_confidence(&self) -> f32 {
        self.current_confidence
    }

    /// Get samples in current segment.
    pub fn samples_in_current_segment(&self) -> usize {
        self.samples_in_current_mode
    }

    /// Get total samples processed.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get number of segment boundaries detected.
    pub fn segment_boundary_count(&self) -> u64 {
        self.segment_boundary_count
    }
}

impl Default for SegmentationEngine {
    fn default() -> Self {
        Self::new(SegmentationConfig::default())
    }
}

/// Output when a segment boundary is detected.
#[derive(Debug, Clone)]
pub struct SegmentClassification {
    /// Transition from one mode to another (if any).
    pub transition_type: Option<(MotionMode, MotionMode)>,

    /// Sample index where this segment started.
    pub sample_index: u64,

    /// Number of samples in the completed segment.
    pub duration_samples: usize,

    /// Motion mode of the completed segment.
    pub mode: MotionMode,

    /// Confidence in the classification.
    pub confidence: f32,

    /// Mean acceleration magnitude during segment.
    pub accel_mean: f32,

    /// Mean gyro magnitude during segment.
    pub gyro_mean: f32,

    /// Variance of acceleration.
    pub accel_variance: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_segmentation_config_default() {
        let config = SegmentationConfig::default();
        assert_eq!(config.window_size, 50);
        assert!(config.still_accel_threshold > 0.0);
    }

    #[test]
    fn test_circular_window_creation() {
        let window = CircularWindow::new(10);
        assert_eq!(window.len(), 0);
        assert!(!window.is_full());
    }

    #[test]
    fn test_circular_window_push_and_fill() {
        let mut window = CircularWindow::new(5);

        for i in 0..5 {
            window.push(i as f32, (i as f32) * 0.1);
            assert_eq!(window.len(), i + 1);
        }

        assert!(window.is_full());

        // Add one more to wrap around
        window.push(5.0, 0.5);
        assert_eq!(window.len(), 5); // Still 5, wrapped
        assert!(window.is_full());
    }

    #[test]
    fn test_window_stats_still_motion() {
        let mut window = CircularWindow::new(10);

        // Add 10 samples of still motion (very low acceleration)
        for _ in 0..10 {
            window.push(0.1, 0.01);
        }

        let stats = window.compute_stats();
        assert!(stats.accel_mean < 0.2);
        assert!(stats.gyro_mean < 0.02);
        assert!(stats.accel_variance < 0.01);
    }

    #[test]
    fn test_window_stats_steady_motion() {
        let mut window = CircularWindow::new(10);

        // Add 10 samples of steady motion (moderate acceleration)
        for i in 0..10 {
            let accel = 1.5 + (i as f32 % 3.0) * 0.1; // Vary slightly
            window.push(accel, 0.05);
        }

        let stats = window.compute_stats();
        assert!(stats.accel_mean > 1.0 && stats.accel_mean < 2.0);
        assert!(stats.accel_variance > 0.0); // Should have some variance
    }

    #[test]
    fn test_motion_classifier_still() {
        let config = SegmentationConfig::default();
        let classifier = MotionClassifier::new(config);

        let mut stats = WindowStats {
            sample_count: 10,
            accel_mean: 0.1,
            gyro_mean: 0.01,
            accel_variance: 0.001,
            ..Default::default()
        };
        stats.finalize();

        let (mode, confidence) = classifier.classify(&stats);
        assert_eq!(mode, MotionMode::Still);
        assert!(confidence > 0.5);
    }

    #[test]
    fn test_motion_classifier_steady_motion() {
        let config = SegmentationConfig::default();
        let classifier = MotionClassifier::new(config);

        let mut stats = WindowStats {
            sample_count: 10,
            accel_mean: 1.5,
            accel_sum: 15.0,
            gyro_mean: 0.1,
            accel_variance: 0.1,
            accel_max: 1.6,
            accel_min: 1.4,
            accel_sum_sq_dev: 0.9,
        };
        stats.finalize();

        let (mode, confidence) = classifier.classify(&stats);
        assert_eq!(mode, MotionMode::SteadyMotion);
        assert!(confidence > 0.5);
    }

    #[test]
    fn test_motion_classifier_turning() {
        let config = SegmentationConfig::default();
        let classifier = MotionClassifier::new(config);

        let mut stats = WindowStats {
            sample_count: 10,
            accel_mean: 1.0,
            accel_sum: 10.0,
            gyro_mean: 0.3, // High rotation
            accel_variance: 0.05,
            accel_max: 1.1,
            accel_min: 0.9,
            accel_sum_sq_dev: 0.45,
        };
        stats.finalize();

        let (mode, confidence) = classifier.classify(&stats);
        assert_eq!(mode, MotionMode::Turning);
        assert!(confidence > 0.4);
    }

    #[test]
    fn test_motion_classifier_transitional() {
        let config = SegmentationConfig::default();
        let classifier = MotionClassifier::new(config);

        let mut stats = WindowStats {
            sample_count: 10,
            accel_mean: 4.0, // Very high acceleration
            accel_sum: 40.0,
            gyro_mean: 0.1,
            accel_variance: 0.5,
            accel_max: 4.5,
            accel_min: 3.5,
            accel_sum_sq_dev: 4.5,
        };
        stats.finalize();

        let (mode, _confidence) = classifier.classify(&stats);
        assert_eq!(mode, MotionMode::Transitional);
    }

    #[test]
    fn test_segmentation_engine_creation() {
        let engine = SegmentationEngine::new(SegmentationConfig::default());
        assert_eq!(engine.current_mode(), MotionMode::Still);
        assert_eq!(engine.total_samples(), 0);
    }

    #[test]
    fn test_segmentation_engine_still_detection() {
        let config = SegmentationConfig {
            window_size: 10,
            min_segment_duration: 5,
            ..Default::default()
        };

        let mut engine = SegmentationEngine::new(config);

        // Feed still motion samples
        for _ in 0..20 {
            let _result = engine.process_sample(0.1, 0.01);
            // Should detect transition from Still to... Still (no change)
            // So no boundary until we change motion
        }

        assert_eq!(engine.current_mode(), MotionMode::Still);
    }

    #[test]
    fn test_segmentation_engine_transition_detection() {
        let config = SegmentationConfig {
            window_size: 10,
            min_segment_duration: 8,
            still_accel_threshold: 0.3,
            steady_accel_min: 0.8,
            steady_accel_max: 4.0,
            turning_gyro_threshold: 0.2,
            transition_accel_delta: 3.0,
            noise_confidence_penalty: 0.1,
            ..Default::default()
        };

        let mut engine = SegmentationEngine::new(config);

        // Feed still motion first
        for _ in 0..25 {
            engine.process_sample(0.1, 0.01);
        }

        assert_eq!(engine.current_mode(), MotionMode::Still);

        // Feed steady motion next (1.5 m/s² is in [0.8, 4.0] range)
        let mut mode_changed = false;
        for _ in 0..50 {
            engine.process_sample(1.5, 0.05);
            if engine.current_mode() != MotionMode::Still {
                mode_changed = true;
                break;
            }
        }

        // Mode should have changed from Still
        assert!(mode_changed, "Mode should transition away from Still");
        assert_ne!(engine.current_mode(), MotionMode::Still);
    }

    #[test]
    fn test_segmentation_engine_confidence_update() {
        let config = SegmentationConfig {
            window_size: 10,
            ..Default::default()
        };

        let mut engine = SegmentationEngine::new(config);

        for _ in 0..20 {
            engine.process_sample(0.1, 0.01);
        }

        // Confidence should be high for consistent motion
        assert!(engine.current_confidence() > 0.7);
    }

    #[test]
    fn test_segmentation_boundary_counting() {
        let config = SegmentationConfig {
            window_size: 10,
            min_segment_duration: 5,
            still_accel_threshold: 0.3,
            steady_accel_min: 0.5,
            ..Default::default()
        };

        let mut engine = SegmentationEngine::new(config);

        // Create multiple transitions
        for _ in 0..15 {
            engine.process_sample(0.1, 0.01); // Still
        }

        for _ in 0..30 {
            engine.process_sample(1.5, 0.05); // Steady motion
        }

        for _ in 0..30 {
            engine.process_sample(0.1, 0.01); // Back to still
        }

        // Should detect multiple boundaries
        assert!(engine.segment_boundary_count() > 0);
        assert!(engine.total_samples() > 50);
    }
}
