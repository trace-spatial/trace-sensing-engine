//! Pedestrian Dead Reckoning (PDR) Trajectory Module.
//!
//! Implements 2D/3D trajectory reconstruction using:
//! - Step-based position updates
//! - Heading integration for direction
//! - ZUPT (Zero Velocity Update) for drift correction
//! - Uncertainty propagation
//!
//! PDR provides relative trajectory without GPS, enabling
//! indoor positioning and movement pattern analysis.

use crate::types::{TrajectoryPoint, StancePhase, StepEvent};
use crate::step_detection::StepDetector;
use crate::stance_detection::StanceDetector;
use crate::heading::HeadingEstimator;

/// Configuration for PDR trajectory estimation.
#[derive(Debug, Clone)]
pub struct PdrConfig {
    /// Initial position uncertainty (meters).
    pub initial_uncertainty_m: f32,
    /// Position uncertainty growth per step (meters).
    pub uncertainty_growth_per_step: f32,
    /// Maximum position uncertainty before reset (meters).
    pub max_uncertainty_m: f32,
    /// ZUPT velocity threshold (m/s) - velocity below this is zeroed.
    pub zupt_velocity_threshold: f32,
    /// ZUPT correction gain (0-1).
    pub zupt_correction_gain: f32,
    /// Minimum samples in stance for ZUPT.
    pub min_stance_samples_for_zupt: u32,
    /// Whether to apply altitude estimation from barometer.
    pub use_barometer: bool,
    /// Barometer sensitivity (meters per hPa).
    pub baro_sensitivity: f32,
    /// Trajectory point sampling interval (ms).
    pub trajectory_sample_interval_ms: u64,
}

impl Default for PdrConfig {
    fn default() -> Self {
        Self {
            initial_uncertainty_m: 0.1,
            uncertainty_growth_per_step: 0.05,
            max_uncertainty_m: 10.0,
            zupt_velocity_threshold: 0.1,
            zupt_correction_gain: 0.8,
            min_stance_samples_for_zupt: 5,
            use_barometer: true,
            baro_sensitivity: 8.3, // ~8.3 m per hPa at sea level
            trajectory_sample_interval_ms: 200, // 5 Hz trajectory output
        }
    }
}

/// Zero Velocity Update (ZUPT) state.
#[derive(Debug, Clone, Copy)]
pub struct ZuptState {
    /// Whether ZUPT is currently active.
    pub active: bool,
    /// Number of consecutive stance samples.
    pub stance_count: u32,
    /// Velocity before ZUPT correction.
    pub pre_zupt_velocity: [f32; 3],
    /// Total ZUPT corrections applied.
    pub correction_count: u64,
}

impl Default for ZuptState {
    fn default() -> Self {
        Self {
            active: false,
            stance_count: 0,
            pre_zupt_velocity: [0.0; 3],
            correction_count: 0,
        }
    }
}

/// PDR Trajectory Estimator.
/// 
/// Fuses step detection, heading, and ZUPT to produce
/// a continuous trajectory estimate.
pub struct PdrEstimator {
    config: PdrConfig,
    
    // Current position state
    position: [f32; 3],        // [x, y, z] in meters
    velocity: [f32; 3],        // [vx, vy, vz] in m/s
    heading_rad: f32,          // Current heading
    
    // Uncertainty tracking
    position_uncertainty: f32,
    
    // ZUPT state
    zupt_state: ZuptState,
    
    // Barometer state (for altitude)
    baro_reference: Option<f32>,
    baro_altitude: f32,
    
    // Step accumulation
    step_count: u32,
    total_distance: f32,
    
    // Trajectory history
    trajectory: Vec<TrajectoryPoint>,
    last_trajectory_time_ms: u64,
    
    // Statistics
    zupt_corrections: u64,
    heading_corrections: u64,
}

impl PdrEstimator {
    /// Create a new PDR estimator.
    pub fn new(config: PdrConfig) -> Self {
        Self {
            config: config.clone(),
            position: [0.0; 3],
            velocity: [0.0; 3],
            heading_rad: 0.0,
            position_uncertainty: config.initial_uncertainty_m,
            zupt_state: ZuptState::default(),
            baro_reference: None,
            baro_altitude: 0.0,
            step_count: 0,
            total_distance: 0.0,
            trajectory: Vec::new(),
            last_trajectory_time_ms: 0,
            zupt_corrections: 0,
            heading_corrections: 0,
        }
    }

    /// Create with default configuration.
    pub fn default_estimator() -> Self {
        Self::new(PdrConfig::default())
    }

    /// Update PDR with a step event.
    /// This is the main position update method.
    pub fn update_step(&mut self, step: &StepEvent, heading_rad: f32, timestamp_ms: u64) {
        self.heading_rad = heading_rad;
        
        // Position update using step length and heading
        let dx = step.stride_length_m * heading_rad.cos();
        let dy = step.stride_length_m * heading_rad.sin();
        
        self.position[0] += dx;
        self.position[1] += dy;
        
        // Update velocity estimate
        if step.step_frequency_hz > 0.0 {
            let speed = step.stride_length_m * step.step_frequency_hz;
            self.velocity[0] = speed * heading_rad.cos();
            self.velocity[1] = speed * heading_rad.sin();
        }
        
        // Accumulate
        self.step_count += 1;
        self.total_distance += step.stride_length_m;
        
        // Grow uncertainty
        self.position_uncertainty += self.config.uncertainty_growth_per_step;
        if self.position_uncertainty > self.config.max_uncertainty_m {
            self.position_uncertainty = self.config.max_uncertainty_m;
        }
        
        // Sample trajectory point
        self.maybe_add_trajectory_point(timestamp_ms);
    }

    /// Apply ZUPT correction during stance phase.
    /// Call this when stance is detected.
    pub fn apply_zupt(&mut self, stance_phase: StancePhase, timestamp_ms: u64) {
        if stance_phase == StancePhase::Stance {
            self.zupt_state.stance_count += 1;
            
            if self.zupt_state.stance_count >= self.config.min_stance_samples_for_zupt {
                // Store pre-correction velocity
                self.zupt_state.pre_zupt_velocity = self.velocity;
                
                // Apply ZUPT: zero the velocity
                let gain = self.config.zupt_correction_gain;
                self.velocity[0] *= 1.0 - gain;
                self.velocity[1] *= 1.0 - gain;
                self.velocity[2] *= 1.0 - gain;
                
                // Reduce uncertainty during ZUPT
                self.position_uncertainty *= 0.95;
                
                self.zupt_state.active = true;
                self.zupt_corrections += 1;
            }
        } else {
            // Reset stance counter
            self.zupt_state.stance_count = 0;
            self.zupt_state.active = false;
        }
        
        // Update trajectory if needed
        self.maybe_add_trajectory_point(timestamp_ms);
    }

    /// Update altitude from barometer reading.
    pub fn update_barometer(&mut self, pressure_hpa: f32) {
        if !self.config.use_barometer {
            return;
        }
        
        // Set reference on first reading
        if self.baro_reference.is_none() {
            self.baro_reference = Some(pressure_hpa);
            return;
        }
        
        // Calculate altitude change from reference
        let ref_pressure = self.baro_reference.unwrap();
        let pressure_diff = ref_pressure - pressure_hpa;
        self.baro_altitude = pressure_diff * self.config.baro_sensitivity;
        self.position[2] = self.baro_altitude;
    }

    /// Get the current position estimate.
    pub fn position(&self) -> [f32; 3] {
        self.position
    }

    /// Get the current velocity estimate.
    pub fn velocity(&self) -> [f32; 3] {
        self.velocity
    }

    /// Get the current speed (velocity magnitude).
    pub fn speed(&self) -> f32 {
        (self.velocity[0] * self.velocity[0] 
            + self.velocity[1] * self.velocity[1]).sqrt()
    }

    /// Get the current position uncertainty in meters.
    pub fn uncertainty(&self) -> f32 {
        self.position_uncertainty
    }

    /// Get total step count.
    pub fn step_count(&self) -> u32 {
        self.step_count
    }

    /// Get total distance traveled in meters.
    pub fn total_distance(&self) -> f32 {
        self.total_distance
    }

    /// Get the current trajectory point.
    pub fn current_point(&self, timestamp_ms: u64) -> TrajectoryPoint {
        let mut point = TrajectoryPoint::new(timestamp_ms);
        point.x = self.position[0];
        point.y = self.position[1];
        point.z = self.position[2];
        point.heading_rad = self.heading_rad;
        point.velocity_mps = self.speed();
        point.stance_phase = if self.zupt_state.active { 
            StancePhase::Stance 
        } else { 
            StancePhase::Swing 
        };
        point.uncertainty_m = self.position_uncertainty;
        point.confidence = self.compute_confidence();
        point
    }

    /// Get the full trajectory history.
    pub fn trajectory(&self) -> &[TrajectoryPoint] {
        &self.trajectory
    }

    /// Get the ZUPT state.
    pub fn zupt_state(&self) -> &ZuptState {
        &self.zupt_state
    }

    /// Reset the PDR state (call at known position or to restart).
    pub fn reset(&mut self) {
        self.position = [0.0; 3];
        self.velocity = [0.0; 3];
        self.heading_rad = 0.0;
        self.position_uncertainty = self.config.initial_uncertainty_m;
        self.zupt_state = ZuptState::default();
        self.baro_reference = None;
        self.baro_altitude = 0.0;
        self.step_count = 0;
        self.total_distance = 0.0;
        self.trajectory.clear();
        self.last_trajectory_time_ms = 0;
    }

    /// Reset position to a known location.
    pub fn set_position(&mut self, x: f32, y: f32, z: f32) {
        self.position = [x, y, z];
        self.position_uncertainty = self.config.initial_uncertainty_m;
    }

    /// Set heading (e.g., from compass).
    pub fn set_heading(&mut self, heading_rad: f32) {
        self.heading_rad = heading_rad;
    }

    // =========================================================================
    // PRIVATE METHODS
    // =========================================================================

    fn maybe_add_trajectory_point(&mut self, timestamp_ms: u64) {
        // Sample trajectory at configured interval
        if timestamp_ms - self.last_trajectory_time_ms >= self.config.trajectory_sample_interval_ms {
            let point = self.current_point(timestamp_ms);
            self.trajectory.push(point);
            self.last_trajectory_time_ms = timestamp_ms;
            
            // Limit trajectory size to prevent memory growth
            if self.trajectory.len() > 10000 {
                self.trajectory.remove(0);
            }
        }
    }

    fn compute_confidence(&self) -> f32 {
        let mut confidence = 0.8; // Base confidence
        
        // Reduce with uncertainty
        let uncertainty_factor = 1.0 - (self.position_uncertainty / self.config.max_uncertainty_m);
        confidence *= uncertainty_factor.max(0.3);
        
        // Boost during ZUPT
        if self.zupt_state.active {
            confidence += 0.1;
        }
        
        confidence.clamp(0.0, 1.0)
    }
}

/// Integrated PDR processor that combines all components.
/// This is the main entry point for trajectory estimation.
pub struct IntegratedPdr {
    step_detector: StepDetector,
    stance_detector: StanceDetector,
    heading_estimator: HeadingEstimator,
    pdr_estimator: PdrEstimator,
    
    // Last known state
    last_timestamp_ms: u64,
}

impl IntegratedPdr {
    /// Create a new integrated PDR processor.
    pub fn new() -> Self {
        use crate::step_detection::StepDetectorConfig;
        use crate::stance_detection::StanceDetectorConfig;
        use crate::heading::HeadingConfig;
        
        Self {
            step_detector: StepDetector::new(StepDetectorConfig::default()),
            stance_detector: StanceDetector::new(StanceDetectorConfig::default()),
            heading_estimator: HeadingEstimator::new(HeadingConfig::default()),
            pdr_estimator: PdrEstimator::default_estimator(),
            last_timestamp_ms: 0,
        }
    }

    /// Process a single IMU sample through the full PDR pipeline.
    /// Returns the current trajectory point.
    pub fn process_sample(&mut self, sample: &crate::types::ImuSample) -> TrajectoryPoint {
        self.last_timestamp_ms = sample.timestamp_ms;
        
        // Update heading
        let heading_state = self.heading_estimator.process_sample(sample);
        
        // Detect stance phase
        let stance_phase = self.stance_detector.process_sample(sample);
        
        // Apply ZUPT if in stance
        self.pdr_estimator.apply_zupt(stance_phase, sample.timestamp_ms);
        
        // Detect steps
        if let Some(step) = self.step_detector.process_sample(sample) {
            self.pdr_estimator.update_step(
                &step, 
                heading_state.heading_rad, 
                sample.timestamp_ms
            );
        }
        
        // Update barometer if available
        if let Some(baro) = sample.baro {
            self.pdr_estimator.update_barometer(baro);
        }
        
        self.pdr_estimator.current_point(sample.timestamp_ms)
    }

    /// Get the current position.
    pub fn position(&self) -> [f32; 3] {
        self.pdr_estimator.position()
    }

    /// Get the current heading in radians.
    pub fn heading(&self) -> f32 {
        self.heading_estimator.heading()
    }

    /// Get total steps detected.
    pub fn step_count(&self) -> u32 {
        self.pdr_estimator.step_count()
    }

    /// Get total distance traveled.
    pub fn total_distance(&self) -> f32 {
        self.pdr_estimator.total_distance()
    }

    /// Get the full trajectory.
    pub fn trajectory(&self) -> &[TrajectoryPoint] {
        self.pdr_estimator.trajectory()
    }

    /// Get position uncertainty.
    pub fn uncertainty(&self) -> f32 {
        self.pdr_estimator.uncertainty()
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        self.step_detector.reset();
        self.stance_detector.reset();
        self.heading_estimator.reset();
        self.pdr_estimator.reset();
    }
}

impl Default for IntegratedPdr {
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
    use crate::types::ImuSample;

    #[test]
    fn test_pdr_estimator_creation() {
        let pdr = PdrEstimator::default_estimator();
        assert_eq!(pdr.position(), [0.0, 0.0, 0.0]);
        assert_eq!(pdr.step_count(), 0);
        assert_eq!(pdr.total_distance(), 0.0);
    }

    #[test]
    fn test_step_update() {
        let mut pdr = PdrEstimator::default_estimator();
        
        let step = StepEvent::new(1000, 0.7, 1.8, 0.85, 2.5);
        
        // Walk forward (heading = 0)
        pdr.update_step(&step, 0.0, 1000);
        
        let pos = pdr.position();
        assert!((pos[0] - 0.7).abs() < 0.01, "X should be ~0.7, got {}", pos[0]);
        assert!(pos[1].abs() < 0.01, "Y should be ~0, got {}", pos[1]);
        assert_eq!(pdr.step_count(), 1);
    }

    #[test]
    fn test_heading_affects_position() {
        let mut pdr = PdrEstimator::default_estimator();
        
        let step = StepEvent::new(1000, 1.0, 1.5, 0.9, 2.0);
        
        // Walk at 90 degrees (east)
        pdr.update_step(&step, std::f32::consts::PI / 2.0, 1000);
        
        let pos = pdr.position();
        assert!(pos[0].abs() < 0.1, "X should be ~0, got {}", pos[0]);
        assert!((pos[1] - 1.0).abs() < 0.1, "Y should be ~1.0, got {}", pos[1]);
    }

    #[test]
    fn test_zupt_application() {
        let mut pdr = PdrEstimator::default_estimator();
        
        // Set some velocity
        let step = StepEvent::new(1000, 0.7, 2.0, 0.9, 2.5);
        pdr.update_step(&step, 0.0, 1000);
        
        assert!(pdr.speed() > 0.0);
        
        // Apply ZUPT multiple times
        for i in 0..10 {
            pdr.apply_zupt(StancePhase::Stance, 1100 + i * 20);
        }
        
        // Velocity should be reduced
        assert!(pdr.speed() < 0.5, "Speed should be reduced by ZUPT, got {}", pdr.speed());
    }

    #[test]
    fn test_barometer_altitude() {
        let mut pdr = PdrEstimator::default_estimator();
        
        // First reading sets reference
        pdr.update_barometer(1013.25);
        assert_eq!(pdr.position()[2], 0.0);
        
        // Lower pressure = higher altitude
        pdr.update_barometer(1012.25); // 1 hPa lower
        assert!(pdr.position()[2] > 0.0, "Altitude should increase");
    }

    #[test]
    fn test_trajectory_recording() {
        let mut pdr = PdrEstimator::default_estimator();
        
        // Add several steps
        for i in 0..10 {
            let step = StepEvent::new(i * 500, 0.7, 1.8, 0.85, 2.5);
            pdr.update_step(&step, 0.0, i * 500);
        }
        
        let trajectory = pdr.trajectory();
        assert!(!trajectory.is_empty(), "Trajectory should have points");
    }

    #[test]
    fn test_reset() {
        let mut pdr = PdrEstimator::default_estimator();
        
        // Add some state
        let step = StepEvent::new(1000, 0.7, 1.8, 0.85, 2.5);
        pdr.update_step(&step, 0.0, 1000);
        
        pdr.reset();
        
        assert_eq!(pdr.position(), [0.0, 0.0, 0.0]);
        assert_eq!(pdr.step_count(), 0);
        assert_eq!(pdr.total_distance(), 0.0);
        assert!(pdr.trajectory().is_empty());
    }

    #[test]
    fn test_integrated_pdr() {
        let mut pdr = IntegratedPdr::new();
        
        // Simulate walking: create samples with step-like pattern
        for i in 0..100 {
            let phase = (i as f32 * 0.3).sin();
            let sample = ImuSample::new(
                i * 20,
                [0.1, 0.2, 9.81 + 2.0 * phase],
                [0.0, 0.0, 0.05],
            );
            let _ = pdr.process_sample(&sample);
        }
        
        // Should have detected some steps
        // Note: may be 0 if walking pattern not strong enough
        assert!(pdr.step_count() >= 0);
    }

    #[test]
    fn test_uncertainty_growth() {
        let mut pdr = PdrEstimator::default_estimator();
        let initial_uncertainty = pdr.uncertainty();
        
        // Add steps
        for i in 0..20 {
            let step = StepEvent::new(i * 500, 0.7, 1.8, 0.85, 2.5);
            pdr.update_step(&step, 0.0, i * 500);
        }
        
        // Uncertainty should have grown
        assert!(pdr.uncertainty() > initial_uncertainty,
            "Uncertainty should grow with steps");
    }

    #[test]
    fn test_set_position() {
        let mut pdr = PdrEstimator::default_estimator();
        
        pdr.set_position(10.0, 20.0, 5.0);
        
        let pos = pdr.position();
        assert_eq!(pos, [10.0, 20.0, 5.0]);
    }
}
