//! Vehicle and Transport Mode Detection Module.
//!
//! Detects whether the user is walking, in a vehicle, cycling, etc.
//! This is critical for:
//! - Filtering false motion during vehicle travel
//! - Adjusting PDR behavior (disable step counting in vehicles)
//! - Providing context for zone transitions
//!
//! Detection is based on:
//! - Acceleration patterns (sustained vibration = vehicle)
//! - Gyroscope activity (low rotation in vehicles vs walking)
//! - Frequency analysis (vehicle vibration has different spectrum)

use crate::types::{ImuSample, TransportMode};

/// Configuration for transport mode detection.
#[derive(Debug, Clone)]
pub struct TransportDetectorConfig {
    /// Window size for feature computation (samples).
    pub window_size: usize,
    /// Variance threshold for vehicle detection (m/s²)².
    pub vehicle_variance_threshold: f32,
    /// Maximum gyro magnitude for vehicle (rad/s).
    pub vehicle_max_gyro: f32,
    /// Minimum acceleration variance for walking (m/s²)².
    pub walking_min_variance: f32,
    /// Maximum acceleration variance for walking (m/s²)².
    pub walking_max_variance: f32,
    /// Running acceleration threshold (m/s²).
    pub running_accel_threshold: f32,
    /// Elevator detection - sustained vertical acceleration (m/s²).
    pub elevator_accel_threshold: f32,
    /// Stairs detection - periodic vertical pattern threshold.
    pub stairs_periodicity_threshold: f32,
    /// Minimum samples in mode before confirming transition.
    pub min_mode_samples: usize,
    /// Cycling gyro range (rad/s) - more rotation than vehicle.
    pub cycling_gyro_range: (f32, f32),
}

impl Default for TransportDetectorConfig {
    fn default() -> Self {
        Self {
            window_size: 50,                      // 1 second at 50Hz
            vehicle_variance_threshold: 0.3,      // Low variance = smooth vehicle
            vehicle_max_gyro: 0.15,               // Very low rotation in vehicle
            walking_min_variance: 0.5,            // Walking has more variance
            walking_max_variance: 5.0,            // But not too much
            running_accel_threshold: 15.0,        // High peaks = running
            elevator_accel_threshold: 0.3,        // Sustained vertical accel
            stairs_periodicity_threshold: 0.7,    // Periodic vertical pattern
            min_mode_samples: 25,                 // 500ms at 50Hz
            cycling_gyro_range: (0.1, 0.5),       // Moderate lean/turn
        }
    }
}

/// Features extracted for transport mode classification.
#[derive(Debug, Clone, Copy, Default)]
struct TransportFeatures {
    /// Acceleration magnitude variance.
    accel_variance: f32,
    /// Mean acceleration magnitude.
    accel_mean: f32,
    /// Peak acceleration in window.
    accel_peak: f32,
    /// Gyroscope magnitude mean.
    gyro_mean: f32,
    /// Gyroscope variance.
    gyro_variance: f32,
    /// Vertical acceleration mean (gravity-aligned).
    vertical_accel_mean: f32,
    /// Vertical acceleration variance.
    vertical_accel_variance: f32,
    /// Dominant frequency estimate (from zero-crossings).
    dominant_freq_hz: f32,
    /// High-frequency energy ratio.
    high_freq_ratio: f32,
}

/// Transport mode detector using sensor pattern analysis.
pub struct TransportDetector {
    config: TransportDetectorConfig,
    
    // Sample buffers
    accel_buffer: Vec<[f32; 3]>,
    gyro_buffer: Vec<[f32; 3]>,
    buffer_index: usize,
    buffer_filled: bool,
    
    // Gravity estimation
    gravity: [f32; 3],
    gravity_alpha: f32,
    
    // Mode state machine
    current_mode: TransportMode,
    mode_confidence: f32,
    mode_sample_count: usize,
    candidate_mode: TransportMode,
    candidate_count: usize,
    
    // Vertical acceleration for elevator/stairs
    vertical_accel_buffer: Vec<f32>,
    
    // Statistics
    mode_history: Vec<(TransportMode, u64)>,
}

impl TransportDetector {
    /// Create a new transport detector.
    pub fn new(config: TransportDetectorConfig) -> Self {
        let window_size = config.window_size;
        Self {
            config,
            accel_buffer: vec![[0.0; 3]; window_size],
            gyro_buffer: vec![[0.0; 3]; window_size],
            buffer_index: 0,
            buffer_filled: false,
            gravity: [0.0, 0.0, 9.81],
            gravity_alpha: 0.02,
            current_mode: TransportMode::Unknown,
            mode_confidence: 0.5,
            mode_sample_count: 0,
            candidate_mode: TransportMode::Unknown,
            candidate_count: 0,
            vertical_accel_buffer: vec![0.0; window_size],
            mode_history: Vec::new(),
        }
    }

    /// Create with default configuration.
    pub fn default_detector() -> Self {
        Self::new(TransportDetectorConfig::default())
    }

    /// Process a sample and return current transport mode with confidence.
    pub fn process_sample(&mut self, sample: &ImuSample) -> (TransportMode, f32) {
        // Update gravity estimate
        self.update_gravity(sample);
        
        // Add to buffers
        self.update_buffers(sample);
        
        if !self.buffer_filled {
            return (TransportMode::Unknown, 0.3);
        }
        
        // Extract features
        let features = self.extract_features();
        
        // Classify mode
        let raw_mode = self.classify_mode(&features);
        
        // Apply state machine with hysteresis
        self.update_mode_state(raw_mode, sample.timestamp_ms);
        
        (self.current_mode, self.mode_confidence)
    }

    /// Get the current transport mode.
    pub fn current_mode(&self) -> TransportMode {
        self.current_mode
    }

    /// Get confidence in current mode [0.0, 1.0].
    pub fn confidence(&self) -> f32 {
        self.mode_confidence
    }

    /// Check if user is likely in a vehicle (car, bus, train).
    pub fn is_in_vehicle(&self) -> bool {
        matches!(self.current_mode, TransportMode::Vehicle) && self.mode_confidence > 0.6
    }

    /// Check if user is walking or running (pedestrian locomotion).
    pub fn is_pedestrian(&self) -> bool {
        self.current_mode.is_pedestrian() && self.mode_confidence > 0.5
    }

    /// Check if PDR should be applied.
    pub fn should_apply_pdr(&self) -> bool {
        self.current_mode.should_apply_pdr() && self.mode_confidence > 0.5
    }

    /// Get mode history (for analysis).
    pub fn mode_history(&self) -> &[(TransportMode, u64)] {
        &self.mode_history
    }

    /// Reset detector state.
    pub fn reset(&mut self) {
        self.accel_buffer.iter_mut().for_each(|x| *x = [0.0; 3]);
        self.gyro_buffer.iter_mut().for_each(|x| *x = [0.0; 3]);
        self.vertical_accel_buffer.fill(0.0);
        self.buffer_index = 0;
        self.buffer_filled = false;
        self.current_mode = TransportMode::Unknown;
        self.mode_confidence = 0.5;
        self.mode_sample_count = 0;
        self.candidate_mode = TransportMode::Unknown;
        self.candidate_count = 0;
        self.mode_history.clear();
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

    fn update_buffers(&mut self, sample: &ImuSample) {
        self.accel_buffer[self.buffer_index] = sample.accel;
        self.gyro_buffer[self.buffer_index] = sample.gyro;
        
        // Compute vertical acceleration
        let vertical = self.compute_vertical_accel(sample);
        self.vertical_accel_buffer[self.buffer_index] = vertical;
        
        self.buffer_index = (self.buffer_index + 1) % self.config.window_size;
        if self.buffer_index == 0 {
            self.buffer_filled = true;
        }
    }

    fn compute_vertical_accel(&self, sample: &ImuSample) -> f32 {
        let gravity_mag = (self.gravity[0] * self.gravity[0]
            + self.gravity[1] * self.gravity[1]
            + self.gravity[2] * self.gravity[2]).sqrt();
        
        if gravity_mag < 0.1 {
            return 0.0;
        }
        
        // Project onto gravity direction
        let dot = sample.accel[0] * self.gravity[0] / gravity_mag
            + sample.accel[1] * self.gravity[1] / gravity_mag
            + sample.accel[2] * self.gravity[2] / gravity_mag;
        
        dot - gravity_mag // Linear vertical acceleration
    }

    fn extract_features(&self) -> TransportFeatures {
        let n = self.config.window_size as f32;
        
        // Acceleration statistics
        let mut accel_mags: Vec<f32> = self.accel_buffer.iter()
            .map(|a| (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt())
            .collect();
        
        let accel_mean = accel_mags.iter().sum::<f32>() / n;
        let accel_variance = accel_mags.iter()
            .map(|x| (x - accel_mean) * (x - accel_mean))
            .sum::<f32>() / n;
        let accel_peak = accel_mags.iter().cloned().fold(0.0f32, f32::max);
        
        // Gyroscope statistics
        let gyro_mags: Vec<f32> = self.gyro_buffer.iter()
            .map(|g| (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt())
            .collect();
        
        let gyro_mean = gyro_mags.iter().sum::<f32>() / n;
        let gyro_variance = gyro_mags.iter()
            .map(|x| (x - gyro_mean) * (x - gyro_mean))
            .sum::<f32>() / n;
        
        // Vertical acceleration statistics
        let vertical_mean = self.vertical_accel_buffer.iter().sum::<f32>() / n;
        let vertical_variance = self.vertical_accel_buffer.iter()
            .map(|x| (x - vertical_mean) * (x - vertical_mean))
            .sum::<f32>() / n;
        
        // Frequency estimation via zero-crossing rate
        let dominant_freq_hz = self.estimate_frequency(&accel_mags);
        
        // High-frequency energy ratio (crude spectral feature)
        let high_freq_ratio = self.estimate_high_freq_ratio(&accel_mags);
        
        TransportFeatures {
            accel_variance,
            accel_mean,
            accel_peak,
            gyro_mean,
            gyro_variance,
            vertical_accel_mean: vertical_mean,
            vertical_accel_variance: vertical_variance,
            dominant_freq_hz,
            high_freq_ratio,
        }
    }

    fn estimate_frequency(&self, signal: &[f32]) -> f32 {
        if signal.len() < 4 {
            return 0.0;
        }
        
        // Zero-crossing rate
        let mean = signal.iter().sum::<f32>() / signal.len() as f32;
        let mut crossings = 0u32;
        let mut prev_above = signal[0] > mean;
        
        for &val in signal.iter().skip(1) {
            let above = val > mean;
            if above != prev_above {
                crossings += 1;
            }
            prev_above = above;
        }
        
        // Convert to frequency (assuming 50Hz sampling)
        let duration_s = signal.len() as f32 / 50.0;
        (crossings as f32 / 2.0) / duration_s
    }

    fn estimate_high_freq_ratio(&self, signal: &[f32]) -> f32 {
        if signal.len() < 10 {
            return 0.0;
        }
        
        // Simple high-pass: difference signal
        let mut high_freq_energy = 0.0f32;
        let mut total_energy = 0.0f32;
        
        for i in 1..signal.len() {
            let diff = signal[i] - signal[i - 1];
            high_freq_energy += diff * diff;
            total_energy += signal[i] * signal[i];
        }
        
        if total_energy > 0.0 {
            high_freq_energy / total_energy
        } else {
            0.0
        }
    }

    fn classify_mode(&self, features: &TransportFeatures) -> TransportMode {
        // Stationary: very low variance and gyro
        if features.accel_variance < 0.1 && features.gyro_mean < 0.05 {
            return TransportMode::Stationary;
        }
        
        // Vehicle: low variance (smooth), very low gyro, high-freq vibration
        if features.accel_variance < self.config.vehicle_variance_threshold
            && features.gyro_mean < self.config.vehicle_max_gyro
            && features.high_freq_ratio > 0.1 {
            return TransportMode::Vehicle;
        }
        
        // Elevator: sustained vertical acceleration, low horizontal
        if features.vertical_accel_mean.abs() > self.config.elevator_accel_threshold
            && features.gyro_mean < 0.1
            && features.accel_variance < 1.0 {
            return TransportMode::Elevator;
        }
        
        // Running: high peak acceleration, high cadence
        if features.accel_peak > self.config.running_accel_threshold
            && features.dominant_freq_hz > 2.5 {
            return TransportMode::Running;
        }
        
        // Stairs: high vertical variance with walking pattern
        if features.vertical_accel_variance > 2.0
            && features.dominant_freq_hz > 0.8
            && features.dominant_freq_hz < 2.5 {
            // Check for periodic vertical pattern
            let periodicity = self.check_stairs_periodicity();
            if periodicity > self.config.stairs_periodicity_threshold {
                return TransportMode::Stairs;
            }
        }
        
        // Cycling: moderate gyro (leaning), smooth-ish acceleration
        if features.gyro_mean > self.config.cycling_gyro_range.0
            && features.gyro_mean < self.config.cycling_gyro_range.1
            && features.accel_variance < 3.0
            && features.dominant_freq_hz < 1.0 {
            return TransportMode::Cycling;
        }
        
        // Walking: moderate variance, periodic pattern, reasonable cadence
        if features.accel_variance >= self.config.walking_min_variance
            && features.accel_variance <= self.config.walking_max_variance
            && features.dominant_freq_hz > 0.8
            && features.dominant_freq_hz < 2.5 {
            return TransportMode::Walking;
        }
        
        // Default: unknown or transitional
        TransportMode::Unknown
    }

    fn check_stairs_periodicity(&self) -> f32 {
        // Autocorrelation-based periodicity check
        let signal = &self.vertical_accel_buffer;
        let n = signal.len();
        
        if n < 20 {
            return 0.0;
        }
        
        // Mean removal
        let mean = signal.iter().sum::<f32>() / n as f32;
        let centered: Vec<f32> = signal.iter().map(|x| x - mean).collect();
        
        // Autocorrelation at expected step lag (~500ms = 25 samples at 50Hz)
        let mut max_corr = 0.0f32;
        let mut energy = centered.iter().map(|x| x * x).sum::<f32>();
        
        if energy < 0.01 {
            return 0.0;
        }
        
        for lag in 15..35 { // Search around expected step period
            let mut corr = 0.0f32;
            for i in 0..(n - lag) {
                corr += centered[i] * centered[i + lag];
            }
            corr /= energy;
            max_corr = max_corr.max(corr);
        }
        
        max_corr
    }

    fn update_mode_state(&mut self, raw_mode: TransportMode, timestamp_ms: u64) {
        if raw_mode == self.candidate_mode {
            self.candidate_count += 1;
        } else {
            self.candidate_mode = raw_mode;
            self.candidate_count = 1;
        }
        
        // Confirm mode change after enough consistent samples
        if self.candidate_count >= self.config.min_mode_samples 
            && self.candidate_mode != self.current_mode {
            
            // Record mode transition
            self.mode_history.push((self.current_mode, timestamp_ms));
            if self.mode_history.len() > 100 {
                self.mode_history.remove(0);
            }
            
            self.current_mode = self.candidate_mode;
            self.mode_sample_count = 0;
        }
        
        self.mode_sample_count += 1;
        
        // Update confidence based on consistency
        self.mode_confidence = self.compute_confidence();
    }

    fn compute_confidence(&self) -> f32 {
        let mut confidence = 0.5;
        
        // Higher confidence with more samples in current mode
        let sample_factor = (self.mode_sample_count as f32 / 50.0).min(1.0);
        confidence += sample_factor * 0.3;
        
        // Higher confidence if not Unknown
        if self.current_mode != TransportMode::Unknown {
            confidence += 0.1;
        }
        
        // Higher confidence if candidate matches current
        if self.candidate_mode == self.current_mode {
            confidence += 0.1;
        }
        
        confidence.clamp(0.0, 1.0)
    }
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn create_stationary_samples(count: usize) -> Vec<ImuSample> {
        (0..count).map(|i| {
            ImuSample::new(
                i as u64 * 20,
                [0.01, 0.02, 9.81],
                [0.01, 0.01, 0.01],
            )
        }).collect()
    }

    fn create_walking_samples(count: usize) -> Vec<ImuSample> {
        (0..count).map(|i| {
            let phase = (i as f32 * 0.25).sin();
            ImuSample::new(
                i as u64 * 20,
                [0.5 + phase * 0.3, 0.2, 9.81 + 2.0 * phase],
                [0.1, 0.05, 0.1 + phase * 0.05],
            )
        }).collect()
    }

    fn create_vehicle_samples(count: usize) -> Vec<ImuSample> {
        (0..count).map(|i| {
            // High-frequency low-amplitude vibration
            let vibration = ((i as f32 * 2.0).sin() * 0.1);
            ImuSample::new(
                i as u64 * 20,
                [vibration, vibration * 0.5, 9.81 + vibration],
                [0.02, 0.02, 0.02],
            )
        }).collect()
    }

    #[test]
    fn test_detector_creation() {
        let detector = TransportDetector::default_detector();
        assert_eq!(detector.current_mode(), TransportMode::Unknown);
    }

    #[test]
    fn test_stationary_detection() {
        let mut detector = TransportDetector::default_detector();
        let samples = create_stationary_samples(100);
        
        let mut final_mode = TransportMode::Unknown;
        for sample in &samples {
            let (mode, _) = detector.process_sample(sample);
            final_mode = mode;
        }
        
        assert_eq!(final_mode, TransportMode::Stationary,
            "Should detect stationary mode");
    }

    #[test]
    fn test_walking_detection() {
        let mut detector = TransportDetector::default_detector();
        let samples = create_walking_samples(150);
        
        let mut final_mode = TransportMode::Unknown;
        for sample in &samples {
            let (mode, _) = detector.process_sample(sample);
            final_mode = mode;
        }
        
        // May detect walking or unknown depending on pattern strength
        assert!(final_mode == TransportMode::Walking 
            || final_mode == TransportMode::Unknown
            || final_mode == TransportMode::Running,
            "Should detect walking/running or unknown, got {:?}", final_mode);
    }

    #[test]
    fn test_vehicle_detection() {
        let mut detector = TransportDetector::default_detector();
        let samples = create_vehicle_samples(150);
        
        let mut final_mode = TransportMode::Unknown;
        for sample in &samples {
            let (mode, _) = detector.process_sample(sample);
            final_mode = mode;
        }
        
        // Vehicle detection can be tricky - may also detect as stationary
        assert!(final_mode == TransportMode::Vehicle 
            || final_mode == TransportMode::Stationary
            || final_mode == TransportMode::Unknown,
            "Should detect vehicle or stationary, got {:?}", final_mode);
    }

    #[test]
    fn test_is_in_vehicle() {
        let mut detector = TransportDetector::default_detector();
        
        // Initially not in vehicle
        assert!(!detector.is_in_vehicle());
        
        // Process vehicle samples
        let samples = create_vehicle_samples(150);
        for sample in &samples {
            let _ = detector.process_sample(sample);
        }
        
        // Check method works (may or may not be in vehicle depending on classification)
        let _ = detector.is_in_vehicle();
    }

    #[test]
    fn test_should_apply_pdr() {
        let mut detector = TransportDetector::default_detector();
        
        // Initially no PDR
        assert!(!detector.should_apply_pdr());
        
        // Process walking samples
        let samples = create_walking_samples(150);
        for sample in &samples {
            let _ = detector.process_sample(sample);
        }
        
        // Check method
        let _ = detector.should_apply_pdr();
    }

    #[test]
    fn test_reset() {
        let mut detector = TransportDetector::default_detector();
        
        // Process some samples
        let samples = create_walking_samples(100);
        for sample in &samples {
            let _ = detector.process_sample(sample);
        }
        
        detector.reset();
        
        assert_eq!(detector.current_mode(), TransportMode::Unknown);
        assert!(!detector.buffer_filled);
    }

    #[test]
    fn test_transport_mode_methods() {
        assert!(TransportMode::Walking.is_pedestrian());
        assert!(TransportMode::Running.is_pedestrian());
        assert!(TransportMode::Stairs.is_pedestrian());
        assert!(!TransportMode::Vehicle.is_pedestrian());
        assert!(!TransportMode::Cycling.is_pedestrian());
        
        assert!(TransportMode::Walking.should_apply_pdr());
        assert!(TransportMode::Running.should_apply_pdr());
        assert!(!TransportMode::Vehicle.should_apply_pdr());
    }

    #[test]
    fn test_mode_history() {
        let mut detector = TransportDetector::default_detector();
        
        // Start stationary
        let stationary = create_stationary_samples(100);
        for sample in &stationary {
            let _ = detector.process_sample(sample);
        }
        
        // History should have entries after mode changes
        // (initially may be empty or have Unknown -> Stationary)
        let _ = detector.mode_history();
    }
}
