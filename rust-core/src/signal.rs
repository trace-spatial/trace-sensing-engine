//! Signal filtering and gravity separation.
//!
//! This module provides low-level sensor signal processing:
//! - Gravity vector estimation using incremental low-pass filtering
//! - Acceleration magnitude extraction (motion signal)
//! - Sensor health assessment based on signal consistency
//!
//! Design note: All filters use incremental updates (O(1) per sample).
//! No batch processing, no allocations in hot paths.
//!
//! Why this matters:
//! Phone orientation (pocket, hand, bag, upside-down) must not affect
//! motion evidence. By separating gravity, we achieve orientation invariance.

use crate::types::{ImuSample, SensorHealth};

/// Parameters for signal filtering.
///
/// These values are tuned for mobile sensors (50Hz baseline) and balance
/// filter responsiveness with stability. Conservative by design.
#[derive(Debug, Clone)]
pub struct FilterConfig {
    /// Low-pass filter coefficient for gravity estimation.
    /// Controls how quickly gravity adapts to orientation changes.
    /// Range: [0.0, 1.0]. Typical: 0.02 (5Hz cutoff at 50Hz sampling).
    /// Lower = more stable, slower to adapt. Higher = faster, noisier.
    pub gravity_filter_alpha: f32,

    /// Accelerometer noise threshold in m/s².
    /// Raw sensor variance above this suggests device instability or noise.
    /// Typical: 0.2 m/s² for mobile phones.
    pub accel_noise_threshold: f32,

    /// Gyroscope magnitude threshold in rad/s for significant rotation.
    /// Rotations below this are considered measurement noise.
    /// Typical: 0.05 rad/s.
    pub gyro_noise_threshold: f32,

    /// Maximum expected gravity magnitude in m/s² (typically near 9.81).
    /// Used for sanity checks. Normal range: [9.0, 10.0].
    pub gravity_magnitude_expected: f32,

    /// Tolerance around expected gravity for sensor health assessment.
    /// Range: [0.5, 2.0] m/s². Typical: 1.0.
    pub gravity_magnitude_tolerance: f32,
}

impl Default for FilterConfig {
    fn default() -> Self {
        Self {
            gravity_filter_alpha: 0.02,
            accel_noise_threshold: 0.2,
            gyro_noise_threshold: 0.05,
            gravity_magnitude_expected: 9.81,
            gravity_magnitude_tolerance: 1.0,
        }
    }
}

/// State of the gravity estimation filter.
///
/// Maintains incremental state for O(1) updates. No history buffer needed.
#[derive(Debug, Clone)]
pub struct GravityEstimator {
    /// Current estimated gravity vector [x, y, z] in m/s².
    gravity: [f32; 3],

    /// Configuration for this estimator.
    config: FilterConfig,

    /// Number of samples processed (for debugging).
    sample_count: u64,
}

impl GravityEstimator {
    /// Create a new gravity estimator.
    ///
    /// Initializes gravity to [0, 0, -9.81] (down in global frame).
    /// The filter will adapt to the actual device orientation.
    pub fn new(config: FilterConfig) -> Self {
        Self {
            gravity: [0.0, 0.0, -config.gravity_magnitude_expected],
            config,
            sample_count: 0,
        }
    }

    /// Update gravity estimate with a new accelerometer sample.
    ///
    /// Uses incremental low-pass filter: g_new = α*a + (1-α)*g_old
    /// This runs O(1) and is suitable for real-time mobile processing.
    pub fn update(&mut self, accel: [f32; 3]) {
        let alpha = self.config.gravity_filter_alpha;

        self.gravity[0] = alpha * accel[0] + (1.0 - alpha) * self.gravity[0];
        self.gravity[1] = alpha * accel[1] + (1.0 - alpha) * self.gravity[1];
        self.gravity[2] = alpha * accel[2] + (1.0 - alpha) * self.gravity[2];

        self.sample_count += 1;
    }

    /// Get the current gravity estimate.
    pub fn gravity(&self) -> [f32; 3] {
        self.gravity
    }

    /// Get the gravity magnitude in m/s².
    pub fn gravity_magnitude(&self) -> f32 {
        let x2 = self.gravity[0] * self.gravity[0];
        let y2 = self.gravity[1] * self.gravity[1];
        let z2 = self.gravity[2] * self.gravity[2];
        (x2 + y2 + z2).sqrt()
    }

    /// Get the number of samples processed.
    pub fn sample_count(&self) -> u64 {
        self.sample_count
    }

    /// Assess sensor health based on gravity stability.
    ///
    /// Returns Nominal if gravity magnitude is close to expected.
    /// Returns Degraded if slightly off (device may be in unusual orientation).
    /// Returns Critical if severely off (likely a sensor failure).
    pub fn assess_health(&self) -> SensorHealth {
        let actual_mag = self.gravity_magnitude();
        let expected = self.config.gravity_magnitude_expected;
        let tolerance = self.config.gravity_magnitude_tolerance;

        let delta = (actual_mag - expected).abs();

        if delta < tolerance / 2.0 {
            SensorHealth::Nominal
        } else if delta < tolerance {
            SensorHealth::Degraded
        } else {
            SensorHealth::Critical
        }
    }

    /// Extract linear acceleration by removing gravity.
    ///
    /// Returns: linear_accel = measured_accel - gravity_estimate
    /// This gives us the "true" acceleration independent of phone orientation.
    pub fn extract_linear_accel(&self, measured_accel: [f32; 3]) -> [f32; 3] {
        [
            measured_accel[0] - self.gravity[0],
            measured_accel[1] - self.gravity[1],
            measured_accel[2] - self.gravity[2],
        ]
    }

    /// Compute linear acceleration magnitude (motion signal).
    ///
    /// This is the "true" acceleration magnitude after removing gravity.
    pub fn linear_accel_magnitude(&self, measured_accel: [f32; 3]) -> f32 {
        let linear = self.extract_linear_accel(measured_accel);
        let x2 = linear[0] * linear[0];
        let y2 = linear[1] * linear[1];
        let z2 = linear[2] * linear[2];
        (x2 + y2 + z2).sqrt()
    }
}

/// A simple signal quality assessor.
///
/// Tracks whether the sensor signal is stable enough for feature extraction.
#[derive(Debug, Clone)]
pub struct SignalQualityMonitor {
    /// Running variance estimate of linear acceleration.
    running_variance: f32,
    /// Running mean of linear acceleration magnitude.
    running_mean: f32,
    /// Sample count for averaging.
    sample_count: u64,
    /// Configuration.
    config: FilterConfig,
}

impl SignalQualityMonitor {
    /// Create a new signal quality monitor.
    pub fn new(config: FilterConfig) -> Self {
        Self {
            running_variance: 0.0,
            running_mean: 0.0,
            sample_count: 0,
            config,
        }
    }

    /// Update the monitor with a new sample.
    ///
    /// Uses Welford's incremental variance algorithm for numerical stability.
    pub fn update(&mut self, linear_accel_magnitude: f32) {
        self.sample_count += 1;

        let count = self.sample_count as f32;
        let delta = linear_accel_magnitude - self.running_mean;

        self.running_mean += delta / count;

        // Welford's update for variance
        let delta2 = linear_accel_magnitude - self.running_mean;
        self.running_variance += delta * delta2;
    }

    /// Get the current variance estimate.
    pub fn variance(&self) -> f32 {
        if self.sample_count < 2 {
            return 0.0;
        }
        self.running_variance / (self.sample_count as f32 - 1.0)
    }

    /// Get the current mean.
    pub fn mean(&self) -> f32 {
        self.running_mean
    }

    /// Get the current standard deviation.
    pub fn std_dev(&self) -> f32 {
        self.variance().sqrt()
    }

    /// Assess if the signal is too noisy.
    ///
    /// Returns true if variance exceeds threshold (sensor quality degraded).
    pub fn is_too_noisy(&self) -> bool {
        self.variance() > (self.config.accel_noise_threshold * self.config.accel_noise_threshold)
    }

    /// Reset the monitor.
    pub fn reset(&mut self) {
        self.running_variance = 0.0;
        self.running_mean = 0.0;
        self.sample_count = 0;
    }
}

/// Complete signal processor combining gravity separation and quality assessment.
///
/// This is the hot-path component. Must be O(1) per sample with minimal overhead.
pub struct SignalProcessor {
    gravity_estimator: GravityEstimator,
    quality_monitor: SignalQualityMonitor,
    #[allow(dead_code)]
    config: FilterConfig,
}

impl SignalProcessor {
    /// Create a new signal processor with default configuration.
    pub fn new() -> Self {
        let config = FilterConfig::default();
        Self {
            gravity_estimator: GravityEstimator::new(config.clone()),
            quality_monitor: SignalQualityMonitor::new(config.clone()),
            config,
        }
    }

    /// Create a new signal processor with custom configuration.
    pub fn with_config(config: FilterConfig) -> Self {
        Self {
            gravity_estimator: GravityEstimator::new(config.clone()),
            quality_monitor: SignalQualityMonitor::new(config.clone()),
            config,
        }
    }

    /// Process a single IMU sample.
    ///
    /// Returns the processed result with gravity-removed acceleration and health status.
    pub fn process_sample(&mut self, sample: &ImuSample) -> ProcessedSample {
        // Update gravity estimate
        self.gravity_estimator.update(sample.accel);

        // Extract linear acceleration (motion signal)
        let linear_accel = self.gravity_estimator.extract_linear_accel(sample.accel);
        let linear_mag = self.gravity_estimator.linear_accel_magnitude(sample.accel);

        // Track signal quality
        self.quality_monitor.update(linear_mag);

        // Assess sensor health
        let sensor_health = self.gravity_estimator.assess_health();

        ProcessedSample {
            timestamp_ms: sample.timestamp_ms,
            linear_accel,
            linear_accel_magnitude: linear_mag,
            gravity: self.gravity_estimator.gravity(),
            gyro: sample.gyro,
            sensor_health,
            is_noisy: self.quality_monitor.is_too_noisy(),
        }
    }

    /// Get the gravity estimator for diagnostics.
    pub fn gravity_estimator(&self) -> &GravityEstimator {
        &self.gravity_estimator
    }

    /// Get the quality monitor for diagnostics.
    pub fn quality_monitor(&self) -> &SignalQualityMonitor {
        &self.quality_monitor
    }
}

impl Default for SignalProcessor {
    fn default() -> Self {
        Self::new()
    }
}

/// A processed IMU sample after gravity separation.
#[derive(Debug, Clone)]
pub struct ProcessedSample {
    /// Original timestamp.
    pub timestamp_ms: u64,
    /// Linear acceleration (gravity removed) [x, y, z] in m/s².
    pub linear_accel: [f32; 3],
    /// Magnitude of linear acceleration in m/s².
    pub linear_accel_magnitude: f32,
    /// Estimated gravity vector [x, y, z] in m/s².
    pub gravity: [f32; 3],
    /// Gyroscope reading [x, y, z] in rad/s.
    pub gyro: [f32; 3],
    /// Health of the sensor data.
    pub sensor_health: SensorHealth,
    /// Whether the signal is too noisy.
    pub is_noisy: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gravity_estimator_initialization() {
        let config = FilterConfig::default();
        let estimator = GravityEstimator::new(config);

        // Should start with [0, 0, -9.81]
        let gravity = estimator.gravity();
        assert!((gravity[0]).abs() < 0.01);
        assert!((gravity[1]).abs() < 0.01);
        assert!((gravity[2] + 9.81).abs() < 0.01);
    }

    #[test]
    fn test_gravity_filter_convergence() {
        let config = FilterConfig::default();
        let mut estimator = GravityEstimator::new(config);

        // Simulate 100 samples of gravity pointing down
        for _ in 0..100 {
            estimator.update([0.0, 0.0, -9.81]);
        }

        let gravity = estimator.gravity();
        // Should be very close to [0, 0, -9.81]
        assert!(gravity[0].abs() < 0.1);
        assert!(gravity[1].abs() < 0.1);
        assert!((gravity[2] + 9.81).abs() < 0.1);
    }

    #[test]
    fn test_gravity_orientation_invariance() {
        let config = FilterConfig::default();
        let mut estimator = GravityEstimator::new(config);

        // Simulate device rotated 90 degrees: gravity now in x direction
        // With default alpha (0.02), needs ~200 samples to fully converge
        for _ in 0..200 {
            estimator.update([9.81, 0.0, 0.0]);
        }

        let gravity = estimator.gravity();
        // Should adapt to [9.81, 0, 0] with reasonable tolerance
        assert!((gravity[0] - 9.81).abs() < 0.5, "gravity x should be ~9.81");
        assert!(gravity[1].abs() < 0.2, "gravity y should be ~0");
        assert!(gravity[2].abs() < 0.5, "gravity z should be ~0");
    }

    #[test]
    fn test_gravity_magnitude() {
        let config = FilterConfig::default();
        let mut estimator = GravityEstimator::new(config);

        for _ in 0..100 {
            estimator.update([0.0, 0.0, -9.81]);
        }

        let mag = estimator.gravity_magnitude();
        assert!((mag - 9.81).abs() < 0.2);
    }

    #[test]
    fn test_linear_accel_extraction() {
        let config = FilterConfig::default();
        let mut estimator = GravityEstimator::new(config);

        // Converge gravity first
        for _ in 0..100 {
            estimator.update([0.0, 0.0, -9.81]);
        }

        // Now measure [1, 0, -9.81 - 1] = 1 m/s² up + gravity down
        let measured = [1.0, 0.0, -9.81 - 1.0];
        let linear = estimator.extract_linear_accel(measured);

        // Should get approximately [1, 0, -1] (motion up against gravity)
        assert!((linear[0] - 1.0).abs() < 0.2);
        assert!(linear[1].abs() < 0.1);
        assert!((linear[2] + 1.0).abs() < 0.2);
    }

    #[test]
    fn test_sensor_health_assessment() {
        let config = FilterConfig::default();
        let mut estimator = GravityEstimator::new(config);

        // Converge to normal gravity
        for _ in 0..100 {
            estimator.update([0.0, 0.0, -9.81]);
        }
        assert_eq!(estimator.assess_health(), SensorHealth::Nominal);

        // Simulate degraded sensor (gravity off by 0.6 m/s²)
        for _ in 0..100 {
            estimator.update([0.0, 0.0, -9.21]);
        }
        assert_ne!(estimator.assess_health(), SensorHealth::Critical);

        // Simulate critical sensor failure (gravity off by 2 m/s²)
        for _ in 0..100 {
            estimator.update([0.0, 0.0, -11.81]);
        }
        assert_eq!(estimator.assess_health(), SensorHealth::Critical);
    }

    #[test]
    fn test_signal_quality_monitor() {
        let config = FilterConfig::default();
        let mut monitor = SignalQualityMonitor::new(config);

        // Feed it a steady signal (no noise)
        for _ in 0..100 {
            monitor.update(1.0);
        }

        // Variance should be near zero
        assert!(monitor.variance() < 0.01);
        assert!(!monitor.is_too_noisy());
    }

    #[test]
    fn test_signal_quality_noisy_detection() {
        let config = FilterConfig::default();
        let mut monitor = SignalQualityMonitor::new(config);

        // Feed it a noisy signal (alternating between 0 and 1)
        for i in 0..100 {
            let value = if i % 2 == 0 { 1.0 } else { 0.0 };
            monitor.update(value);
        }

        // Variance should be high
        assert!(monitor.variance() > 0.1);
        assert!(monitor.is_too_noisy());
    }

    #[test]
    fn test_signal_processor_full_pipeline() {
        let mut processor = SignalProcessor::new();

        // Create a simple sample
        let sample = ImuSample::new(
            1000,
            [0.0, 0.0, -9.81],  // gravity down
            [0.0, 0.0, 0.0],    // no rotation
        );

        let processed = processor.process_sample(&sample);

        assert_eq!(processed.timestamp_ms, 1000);
        assert!(processed.linear_accel_magnitude < 0.5); // Should be near zero (only gravity)
        assert_eq!(processed.sensor_health, SensorHealth::Nominal);
        assert!(!processed.is_noisy);
    }

    #[test]
    fn test_sample_count() {
        let config = FilterConfig::default();
        let mut estimator = GravityEstimator::new(config);

        assert_eq!(estimator.sample_count(), 0);
        estimator.update([0.0, 0.0, -9.81]);
        assert_eq!(estimator.sample_count(), 1);
        estimator.update([0.0, 0.0, -9.81]);
        assert_eq!(estimator.sample_count(), 2);
    }
}
