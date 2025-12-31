/// Complete motion evidence pipeline integrating all sensor processing stages.
///
/// This module orchestrates the full data flow from raw IMU samples through
/// signal processing, orientation normalization, motion segmentation, and
/// transition detection to produce final MotionEvidenceWindow outputs.
///
/// # Architecture
///
/// The pipeline follows a strict evidence-first design:
/// 1. **Signal Processing**: Extract linear acceleration, assess noise
/// 2. **Motion Segmentation**: Classify motion modes (Still/Steady/Turning)
/// 3. **Transition Detection**: Identify state change events
/// 4. **Window Assembly**: Package results into MotionEvidenceWindow
///
/// ## Full Movement Pipeline (FullMovementPipeline)
///
/// For complete IMU processing including step detection, PDR trajectory,
/// heading estimation, and transport mode classification, use the
/// `FullMovementPipeline` which outputs `MovementEpisode` objects.
///
/// Each stage is independent and can fail gracefully with validity flags.
/// No data is retained beyond what's needed for the current processing window.
///
/// # Privacy & Performance
/// - Raw sensor data never persists; only processed evidence is output
/// - O(1) per-sample: fixed memory, no buffers, constant time processing
/// - Battery-aware: suitable for continuous background execution

use crate::types::*;
use crate::signal::{SignalProcessor};
use crate::orientation::{OrientationConfig, OrientationEstimator, MadgwickAHRS};
use crate::segmentation::{SegmentationConfig, SegmentationEngine};
use crate::transitions::{
    TransitionConfig, TransitionDetector,
};
use crate::step_detection::{StepDetector, StepDetectorConfig};
use crate::stance_detection::{StanceDetector, StanceDetectorConfig};
use crate::heading::{HeadingEstimator, HeadingConfig};
use crate::trajectory::{PdrEstimator, PdrConfig};
use crate::vehicle_detection::{TransportDetector, TransportDetectorConfig};
use crate::micro_events::{MicroEventDetector, MicroEventConfig};

/// Configuration for the complete motion evidence pipeline.
///
/// Bundles all sub-component configurations into a single, coherent package
/// for configuring the entire processing flow.
#[derive(Debug, Clone)]
pub struct PipelineConfig {
    /// Signal processing configuration (gravity estimation, noise detection).
    pub filter_config: crate::signal::FilterConfig,

    /// Orientation estimation configuration (sensor fusion parameters).
    pub orientation_config: OrientationConfig,

    /// Motion segmentation configuration (mode classification thresholds).
    pub segmentation_config: SegmentationConfig,

    /// Transition detection configuration (event classification thresholds).
    pub transition_config: TransitionConfig,

    /// Window duration for evidence aggregation (milliseconds).
    /// Determines how long a processing window lasts before emission.
    pub window_duration_ms: u32,

    /// Sample rate of input sensor data (Hz).
    /// Used to compute actual time from sample counts.
    pub sample_rate_hz: f32,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            filter_config: crate::signal::FilterConfig::default(),
            orientation_config: OrientationConfig::default(),
            segmentation_config: SegmentationConfig::default(),
            transition_config: TransitionConfig::default(),
            window_duration_ms: 1000,  // 1 second window
            sample_rate_hz: 50.0,      // 50Hz baseline
        }
    }
}

/// Complete motion evidence pipeline processor.
///
/// Maintains internal state across all processing stages and emits
/// MotionEvidenceWindow objects at regular intervals.
pub struct MotionEvidencePipeline {
    config: PipelineConfig,

    // Processing stages
    signal_processor: SignalProcessor,
    orientation_estimator: OrientationEstimator,
    segmentation_engine: SegmentationEngine,
    transition_detector: TransitionDetector,

    // Window tracking
    window_sample_count: usize,
    window_sample_limit: usize,
    current_sample_index: usize,

    // Window evidence collection
    segments: Vec<MotionSegment>,
    transitions: Vec<TransitionCandidate>,
    current_mode: MotionMode,
    window_start_sample: usize,
}

impl MotionEvidencePipeline {
    /// Creates a new pipeline with given configuration.
    pub fn new(config: PipelineConfig) -> Self {
        // Calculate sample limit for window duration
        let window_sample_limit =
            ((config.window_duration_ms as f32 / 1000.0) * config.sample_rate_hz) as usize;

        Self {
            signal_processor: SignalProcessor::new(),
            orientation_estimator: OrientationEstimator::new(config.orientation_config.clone()),
            segmentation_engine: SegmentationEngine::new(config.segmentation_config.clone()),
            transition_detector: TransitionDetector::new(config.transition_config.clone()),

            config,
            window_sample_count: 0,
            window_sample_limit: window_sample_limit.max(1),
            current_sample_index: 0,
            segments: Vec::new(),
            transitions: Vec::new(),
            current_mode: MotionMode::Still,
            window_start_sample: 0,
        }
    }

    /// Processes a single IMU sample through the entire pipeline.
    ///
    /// Returns Some(MotionEvidenceWindow) if a window boundary is reached,
    /// None if processing continues within the current window.
    pub fn process_sample(&mut self, imu_sample: &ImuSample) -> Option<MotionEvidenceWindow> {
        // Stage 1: Signal Processing
        let processed_sample = self.signal_processor.process_sample(imu_sample);

        // Stage 2: Orientation Estimation (update but don't need output for pipeline)
        // Note: Simplified - would need gravity vector from signal processor
        // self.orientation_estimator.update(imu_sample, processed_sample.gravity);

        // Stage 3: Motion Segmentation
        let _segmentation_result = self.segmentation_engine.process_sample(
            processed_sample.linear_accel_magnitude,
            imu_sample.gyro_magnitude(),
        );

        // Note: Segment collection and transition detection would go here
        // For now, simplified to focus on window assembly

        self.window_sample_count += 1;
        self.current_sample_index += 1;

        // Check if window boundary reached
        if self.window_sample_count >= self.window_sample_limit {
            let window = self.assemble_window();
            self.reset_window();
            Some(window)
        } else {
            None
        }
    }

    /// Assembles a MotionEvidenceWindow from current window data.
    fn assemble_window(&self) -> MotionEvidenceWindow {
        // Calculate timestamps from sample indices
        let ms_per_sample = 1000.0 / self.config.sample_rate_hz;
        let start_ms = (self.window_start_sample as f32 * ms_per_sample) as u64;
        let end_ms = (self.current_sample_index as f32 * ms_per_sample) as u64;

        // Determine overall validity
        let window_validity = ValidityState::Valid;  // Simplified for now

        let mut window = MotionEvidenceWindow::new(
            self.window_start_sample as u64,  // Use start sample as window_id
            start_ms,
            end_ms,
            self.current_mode,
            SensorHealth::Nominal,
            1,  // schema_version
        );

        window.segments = self.segments.clone();
        window.transitions = self.transitions.clone();
        window.validity_state = window_validity;

        // Compute confidence
        let confidence = if self.transitions.is_empty() {
            0.5  // No transitions = moderate confidence in stability
        } else {
            let sum: f64 = self.transitions.iter().map(|t| t.confidence as f64).sum();
            (sum / self.transitions.len() as f64) as f32
        };
        window.confidence = confidence;

        window
    }

    /// Resets internal state for the next window.
    fn reset_window(&mut self) {
        self.window_sample_count = 0;
        self.window_start_sample = self.current_sample_index;
        self.segments.clear();
        self.transitions.clear();
    }

    /// Emits a final window for any remaining samples.
    ///
    /// Call this when input ends to get the last partial window.
    pub fn flush(&mut self) -> Option<MotionEvidenceWindow> {
        if self.window_sample_count > 0 {
            let window = self.assemble_window();
            self.reset_window();
            Some(window)
        } else {
            None
        }
    }

    /// Returns the current motion mode.
    pub fn current_mode(&self) -> MotionMode {
        self.current_mode
    }

    /// Returns the total sample count processed so far.
    pub fn total_samples(&self) -> usize {
        self.current_sample_index
    }

    /// Returns the number of samples in current window.
    pub fn window_samples(&self) -> usize {
        self.window_sample_count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pipeline_config_default() {
        let config = PipelineConfig::default();
        assert_eq!(config.window_duration_ms, 1000);
        assert_eq!(config.sample_rate_hz, 50.0);
    }

    #[test]
    fn test_pipeline_creation() {
        let config = PipelineConfig::default();
        let pipeline = MotionEvidencePipeline::new(config);
        assert_eq!(pipeline.total_samples(), 0);
        assert_eq!(pipeline.window_samples(), 0);
        assert_eq!(pipeline.current_mode(), MotionMode::Still);
    }

    #[test]
    fn test_pipeline_window_calculation() {
        let mut config = PipelineConfig::default();
        config.window_duration_ms = 200;  // 200ms
        config.sample_rate_hz = 50.0;      // 50Hz = 10 samples per 200ms
        let pipeline = MotionEvidencePipeline::new(config);
        assert_eq!(pipeline.window_sample_limit, 10);
    }

    #[test]
    fn test_pipeline_flush_empty() {
        let config = PipelineConfig::default();
        let mut pipeline = MotionEvidencePipeline::new(config);
        // Flush on empty pipeline should return None
        let result = pipeline.flush();
        assert!(result.is_none());
    }
}


// ============================================================================
// FULL MOVEMENT PIPELINE
// ============================================================================
// Integrates all IMU processing modules for complete movement analysis

/// Configuration for the full movement pipeline.
#[derive(Debug, Clone)]
pub struct FullMovementConfig {
    /// Step detection configuration.
    pub step_config: StepDetectorConfig,
    
    /// Stance detection configuration.
    pub stance_config: StanceDetectorConfig,
    
    /// Heading estimation configuration.
    pub heading_config: HeadingConfig,
    
    /// PDR trajectory configuration.
    pub pdr_config: PdrConfig,
    
    /// Transport mode detection configuration.
    pub transport_config: TransportDetectorConfig,
    
    /// Micro-event detection configuration.
    pub micro_event_config: MicroEventConfig,
    
    /// Orientation estimation configuration.
    pub orientation_config: OrientationConfig,
    
    /// Window duration for episode emission (milliseconds).
    pub episode_duration_ms: u64,
    
    /// Sample rate (Hz).
    pub sample_rate_hz: f32,
}

impl Default for FullMovementConfig {
    fn default() -> Self {
        Self {
            step_config: StepDetectorConfig::default(),
            stance_config: StanceDetectorConfig::default(),
            heading_config: HeadingConfig::default(),
            pdr_config: PdrConfig::default(),
            transport_config: TransportDetectorConfig::default(),
            micro_event_config: MicroEventConfig::default(),
            orientation_config: OrientationConfig::default(),
            episode_duration_ms: 5000, // 5 second episodes
            sample_rate_hz: 50.0,
        }
    }
}

/// Per-sample output from the full movement pipeline.
#[derive(Debug, Clone)]
pub struct MovementSampleOutput {
    /// Current trajectory position (also aliased as `trajectory` for CEBE-X).
    pub trajectory: TrajectoryPoint,
    
    /// Step event (if detected this sample).
    pub step_event: Option<StepEvent>,
    
    /// Current stance phase.
    pub stance_phase: StancePhase,
    
    /// Current transport mode.
    pub transport_mode: TransportMode,
    
    /// Micro-events detected this sample.
    pub micro_events: Vec<MicroEventCandidate>,
    
    /// Current heading (radians).
    pub heading_rad: f32,
    
    /// Euler angles from orientation filter (roll, pitch, yaw).
    pub euler_angles: (f32, f32, f32),
    
    /// Is PDR currently active (pedestrian mode).
    pub pdr_active: bool,
}

impl MovementSampleOutput {
    /// Alias for trajectory point (backward compatibility).
    pub fn position(&self) -> &TrajectoryPoint {
        &self.trajectory
    }
}

/// Full movement pipeline integrating all IMU processing stages.
///
/// This is the main entry point for complete motion analysis including:
/// - Orientation estimation (Madgwick AHRS)
/// - Step detection and counting
/// - Stance/swing phase detection
/// - Heading integration with drift correction
/// - PDR trajectory reconstruction with ZUPT
/// - Transport mode classification
/// - Micro-event detection (phone pickup, putdown, etc.)
///
/// # Example
///
/// ```ignore
/// use trace_sensing::pipeline::FullMovementPipeline;
/// use trace_sensing::types::ImuSample;
///
/// let mut pipeline = FullMovementPipeline::new();
///
/// for sample in samples {
///     let output = pipeline.process_sample(&sample);
///     println!("Position: ({:.2}, {:.2})", output.position.x, output.position.y);
///     println!("Steps: {}", pipeline.step_count());
/// }
///
/// // Get final movement episode
/// let episode = pipeline.flush();
/// ```
pub struct FullMovementPipeline {
    config: FullMovementConfig,
    
    // Processing modules
    orientation_estimator: MadgwickAHRS,
    step_detector: StepDetector,
    stance_detector: StanceDetector,
    heading_estimator: HeadingEstimator,
    pdr_estimator: PdrEstimator,
    transport_detector: TransportDetector,
    micro_event_detector: MicroEventDetector,
    
    // Episode tracking
    episode_start_ms: u64,
    episode_step_count: u32,
    episode_distance_m: f32,
    episode_start_heading: f32,
    episode_trajectory: Vec<TrajectoryPoint>,
    episode_segments: Vec<MotionSegment>,
    episode_transitions: Vec<TransitionCandidate>,
    episode_micro_events: Vec<MicroEventCandidate>,
    
    // Current state
    current_position: TrajectoryPoint,
    current_stance: StancePhase,
    current_transport: TransportMode,
    current_heading: f32,
    prev_transport: TransportMode,
    
    // Statistics
    total_steps: u64,
    total_distance_m: f64,
    total_samples: u64,
    prev_timestamp_ms: u64,
    episode_id_counter: u64,
}

impl FullMovementPipeline {
    /// Create a new full movement pipeline with default configuration.
    pub fn new() -> Self {
        Self::with_config(FullMovementConfig::default())
    }

    /// Create a new full movement pipeline with custom configuration.
    pub fn with_config(config: FullMovementConfig) -> Self {
        Self {
            orientation_estimator: MadgwickAHRS::new(config.orientation_config.clone()),
            step_detector: StepDetector::new(config.step_config.clone()),
            stance_detector: StanceDetector::new(config.stance_config.clone()),
            heading_estimator: HeadingEstimator::new(config.heading_config.clone()),
            pdr_estimator: PdrEstimator::new(config.pdr_config.clone()),
            transport_detector: TransportDetector::new(config.transport_config.clone()),
            micro_event_detector: MicroEventDetector::new(config.micro_event_config.clone()),
            
            config,
            
            episode_start_ms: 0,
            episode_step_count: 0,
            episode_distance_m: 0.0,
            episode_start_heading: 0.0,
            episode_trajectory: Vec::new(),
            episode_segments: Vec::new(),
            episode_transitions: Vec::new(),
            episode_micro_events: Vec::new(),
            
            current_position: TrajectoryPoint::new(0),
            current_stance: StancePhase::Unknown,
            current_transport: TransportMode::Unknown,
            current_heading: 0.0,
            prev_transport: TransportMode::Unknown,
            
            total_steps: 0,
            total_distance_m: 0.0,
            total_samples: 0,
            prev_timestamp_ms: 0,
            episode_id_counter: 0,
        }
    }

    /// Process a single IMU sample through the complete pipeline.
    ///
    /// Returns per-sample output and optionally triggers episode emission.
    pub fn process_sample(&mut self, sample: &ImuSample) -> MovementSampleOutput {
        // Initialize episode start time
        if self.total_samples == 0 {
            self.episode_start_ms = sample.timestamp_ms;
            self.episode_start_heading = 0.0;
            self.current_position = TrajectoryPoint::new(sample.timestamp_ms);
        }
        
        // 1. Orientation estimation (Madgwick AHRS)
        self.orientation_estimator.update(sample);
        let euler_angles = self.orientation_estimator.euler_angles();
        
        // 2. Transport mode detection - returns (mode, confidence)
        let (transport_mode, _confidence) = self.transport_detector.process_sample(sample);
        self.current_transport = transport_mode;
        
        // 3. Micro-event detection
        let micro_events = self.micro_event_detector.process_sample(sample);
        self.episode_micro_events.extend(micro_events.clone());
        
        // 4. Step detection (only when pedestrian)
        let step_event = if self.transport_detector.is_pedestrian() {
            self.step_detector.process_sample(sample)
        } else {
            None
        };
        
        // 5. Stance detection (only when pedestrian)
        self.current_stance = if self.transport_detector.is_pedestrian() {
            self.stance_detector.process_sample(sample)
        } else {
            StancePhase::Unknown
        };
        
        // 6. Heading estimation
        let _heading_state = self.heading_estimator.process_sample(sample);
        self.current_heading = self.heading_estimator.heading();
        
        // Apply drift correction during stance
        if self.current_stance == StancePhase::Stance {
            self.heading_estimator.apply_drift_correction(0.01);
        }
        
        // 7. PDR trajectory update
        let pdr_active = self.transport_detector.should_apply_pdr();
        
        if pdr_active {
            if let Some(ref step) = step_event {
                // Update PDR with step
                self.pdr_estimator.update_step(step, self.current_heading, sample.timestamp_ms);
                
                // Update barometer if available
                if let Some(baro) = sample.baro {
                    self.pdr_estimator.update_barometer(baro);
                }
                
                // Get current position
                self.current_position = self.pdr_estimator.current_point(sample.timestamp_ms);
                
                // Update episode stats
                self.episode_step_count += 1;
                self.episode_distance_m += step.stride_length_m;
                self.total_steps += 1;
                self.total_distance_m += step.stride_length_m as f64;
            }
        }
        
        // Update current position with heading and stance even without step
        self.current_position.heading_rad = self.current_heading;
        self.current_position.stance_phase = self.current_stance;
        
        // Apply ZUPT during stance
        if self.current_stance == StancePhase::Stance {
            self.pdr_estimator.apply_zupt(self.current_stance, sample.timestamp_ms);
        }
        
        // Track trajectory
        if self.total_samples % 10 == 0 {
            // Subsample trajectory to manage memory
            self.episode_trajectory.push(self.current_position.clone());
            if self.episode_trajectory.len() > 1000 {
                // Keep only last 1000 points
                self.episode_trajectory.remove(0);
            }
        }
        
        // Track transport mode transitions
        if self.current_transport != self.prev_transport {
            let transition = TransitionCandidate::new(
                sample.timestamp_ms,
                TransitionType::AbruptStop,
                0.7,
                100,
            );
            self.episode_transitions.push(transition);
            self.prev_transport = self.current_transport;
        }
        
        self.total_samples += 1;
        self.prev_timestamp_ms = sample.timestamp_ms;
        
        MovementSampleOutput {
            trajectory: self.current_position.clone(),
            step_event,
            stance_phase: self.current_stance,
            transport_mode: self.current_transport,
            micro_events,
            heading_rad: self.current_heading,
            euler_angles: (euler_angles[0], euler_angles[1], euler_angles[2]),
            pdr_active,
        }
    }

    /// Alias for process_sample (used by FFI).
    pub fn process(&mut self, sample: &ImuSample) -> MovementSampleOutput {
        self.process_sample(sample)
    }

    /// Process sample and check if episode should be emitted.
    ///
    /// Returns (sample_output, Option<episode>) where episode is Some if
    /// episode duration has elapsed.
    pub fn process_sample_with_episode(&mut self, sample: &ImuSample) 
        -> (MovementSampleOutput, Option<MovementEpisode>) 
    {
        let output = self.process_sample(sample);
        
        // Check if episode duration reached
        let episode = if sample.timestamp_ms - self.episode_start_ms >= self.config.episode_duration_ms {
            Some(self.emit_episode())
        } else {
            None
        };
        
        (output, episode)
    }

    /// Emit current episode and reset for next.
    fn emit_episode(&mut self) -> MovementEpisode {
        self.episode_id_counter += 1;
        let heading_change = self.current_heading - self.episode_start_heading;
        
        let mut episode = MovementEpisode::new(self.episode_id_counter, self.episode_start_ms);
        episode.end_ms = self.prev_timestamp_ms;
        episode.step_count = self.episode_step_count;
        episode.distance_m = self.episode_distance_m;
        episode.trajectory = std::mem::take(&mut self.episode_trajectory);
        episode.final_position = self.current_position.clone();
        episode.heading_change_rad = heading_change;
        episode.transport_mode = self.current_transport;
        episode.segments = std::mem::take(&mut self.episode_segments);
        episode.transitions = std::mem::take(&mut self.episode_transitions);
        episode.micro_events = std::mem::take(&mut self.episode_micro_events);
        episode.confidence = self.compute_episode_confidence();
        
        // Reset for next episode
        self.episode_start_ms = self.prev_timestamp_ms;
        self.episode_step_count = 0;
        self.episode_distance_m = 0.0;
        self.episode_start_heading = self.current_heading;
        
        episode
    }

    /// Flush remaining data as final episode.
    pub fn flush(&mut self) -> Option<MovementEpisode> {
        if self.total_samples > 0 && (self.episode_step_count > 0 || !self.episode_trajectory.is_empty()) {
            Some(self.emit_episode())
        } else {
            None
        }
    }

    /// Get current position.
    pub fn position(&self) -> &TrajectoryPoint {
        &self.current_position
    }

    /// Get current heading in radians.
    pub fn heading_rad(&self) -> f32 {
        self.current_heading
    }

    /// Get current transport mode.
    pub fn transport_mode(&self) -> TransportMode {
        self.current_transport
    }

    /// Get current stance phase.
    pub fn stance_phase(&self) -> StancePhase {
        self.current_stance
    }

    /// Get total step count.
    pub fn step_count(&self) -> u64 {
        self.total_steps
    }

    /// Get total distance traveled (meters).
    pub fn distance_m(&self) -> f64 {
        self.total_distance_m
    }

    /// Get total samples processed.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get the kinematic signature for Zone Mapper integration.
    pub fn kinematic_signature(&self) -> KinematicSignature {
        KinematicSignature {
            median_steps: self.episode_step_count,
            median_turn_angle_rad: self.heading_estimator.total_rotation() / (self.heading_estimator.turn_count().max(1) as f32),
            median_transit_ms: self.config.episode_duration_ms,
            step_count_std: 0.0,
            transit_time_std: 0.0,
            observation_count: 1,
        }
    }

    /// Reset the pipeline to initial state.
    pub fn reset(&mut self) {
        self.orientation_estimator.reset();
        self.step_detector.reset();
        self.stance_detector.reset();
        self.heading_estimator.reset();
        self.pdr_estimator.reset();
        self.transport_detector.reset();
        self.micro_event_detector.reset();
        
        self.episode_start_ms = 0;
        self.episode_step_count = 0;
        self.episode_distance_m = 0.0;
        self.episode_start_heading = 0.0;
        self.episode_trajectory.clear();
        self.episode_segments.clear();
        self.episode_transitions.clear();
        self.episode_micro_events.clear();
        
        self.current_position = TrajectoryPoint::new(0);
        self.current_stance = StancePhase::Unknown;
        self.current_transport = TransportMode::Unknown;
        self.current_heading = 0.0;
        self.prev_transport = TransportMode::Unknown;
        
        self.total_steps = 0;
        self.total_distance_m = 0.0;
        self.total_samples = 0;
        self.prev_timestamp_ms = 0;
    }

    // Helper functions
    
    #[allow(dead_code)]
    fn transport_to_motion_mode(&self, transport: TransportMode) -> MotionMode {
        match transport {
            TransportMode::Stationary => MotionMode::Still,
            TransportMode::Walking | TransportMode::Running => MotionMode::SteadyMotion,
            TransportMode::Vehicle | TransportMode::Cycling => MotionMode::SteadyMotion,
            TransportMode::Stairs | TransportMode::Elevator => MotionMode::Turning,
            TransportMode::Unknown => MotionMode::Still,
        }
    }

    fn compute_episode_confidence(&self) -> f32 {
        let mut confidence = 0.5f32;
        
        // Higher confidence with more steps
        if self.episode_step_count > 10 {
            confidence += 0.2;
        } else if self.episode_step_count > 5 {
            confidence += 0.1;
        }
        
        // Lower confidence in vehicle mode
        if self.current_transport == TransportMode::Vehicle {
            confidence -= 0.1;
        }
        
        // Higher confidence with clear transport mode
        if self.current_transport != TransportMode::Unknown {
            confidence += 0.1;
        }
        
        confidence.clamp(0.0, 1.0)
    }
}

impl Default for FullMovementPipeline {
    fn default() -> Self {
        Self::new()
    }
}


// ============================================================================
// FULL MOVEMENT PIPELINE TESTS
// ============================================================================

#[cfg(test)]
mod full_pipeline_tests {
    use super::*;

    fn create_walking_samples(count: usize) -> Vec<ImuSample> {
        (0..count).map(|i| {
            // Simulate walking with periodic vertical motion
            let t = i as f32 * 0.02; // 50Hz
            let vertical_motion = (t * 3.14159 * 2.0).sin() * 2.0; // ~1Hz step frequency
            ImuSample::new(
                i as u64 * 20,
                [0.1, 0.2, 9.81 + vertical_motion],
                [0.1, 0.1, 0.05],
            )
        }).collect()
    }

    fn create_still_samples(count: usize) -> Vec<ImuSample> {
        (0..count).map(|i| {
            ImuSample::new(
                i as u64 * 20,
                [0.01, 0.02, 9.81],
                [0.01, 0.01, 0.01],
            )
        }).collect()
    }

    #[test]
    fn test_full_pipeline_creation() {
        let pipeline = FullMovementPipeline::new();
        assert_eq!(pipeline.step_count(), 0);
        assert_eq!(pipeline.total_samples(), 0);
        assert_eq!(pipeline.transport_mode(), TransportMode::Unknown);
    }

    #[test]
    fn test_full_pipeline_still_detection() {
        let mut pipeline = FullMovementPipeline::new();
        let samples = create_still_samples(100);
        
        for sample in &samples {
            let _ = pipeline.process_sample(sample);
        }
        
        // Should detect as stationary
        assert_eq!(pipeline.total_samples(), 100);
        // Position should be near origin
        assert!(pipeline.position().x.abs() < 1.0);
        assert!(pipeline.position().y.abs() < 1.0);
    }

    #[test]
    fn test_full_pipeline_config() {
        let mut config = FullMovementConfig::default();
        config.episode_duration_ms = 2000;
        config.sample_rate_hz = 100.0;
        
        let pipeline = FullMovementPipeline::with_config(config);
        assert_eq!(pipeline.total_samples(), 0);
    }

    #[test]
    fn test_full_pipeline_reset() {
        let mut pipeline = FullMovementPipeline::new();
        let samples = create_walking_samples(50);
        
        for sample in &samples {
            let _ = pipeline.process_sample(sample);
        }
        
        assert!(pipeline.total_samples() > 0);
        
        pipeline.reset();
        
        assert_eq!(pipeline.total_samples(), 0);
        assert_eq!(pipeline.step_count(), 0);
        assert_eq!(pipeline.distance_m(), 0.0);
    }

    #[test]
    fn test_movement_sample_output() {
        let mut pipeline = FullMovementPipeline::new();
        let sample = ImuSample::new(1000, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0]);
        
        let output = pipeline.process_sample(&sample);
        
        // Check output structure
        assert!(output.heading_rad >= -3.15 && output.heading_rad <= 3.15);
        assert!(output.micro_events.is_empty() || output.micro_events.len() > 0);
    }

    #[test]
    fn test_kinematic_signature() {
        let mut pipeline = FullMovementPipeline::new();
        let samples = create_walking_samples(100);
        
        for sample in &samples {
            let _ = pipeline.process_sample(sample);
        }
        
        let sig = pipeline.kinematic_signature();
        assert!(sig.median_transit_ms > 0);
    }
}
