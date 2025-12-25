//! Core data types for the IMU Movement Engine.
//!
//! This module defines the fundamental types and structures used throughout
//! the motion evidence extraction pipeline. All types are carefully designed
//! to minimize allocation and maximize clarity.
//!
//! Design principle: Types should make intent obvious. If a concept exists,
//! it gets a type. Never pass raw tuples or untyped collections across boundaries.

/// A single raw inertial measurement unit sample.
///
/// This represents the minimal input contract: three-axis accelerometer,
/// three-axis gyroscope, and a monotonic timestamp. This is never interpreted,
/// only preserved.
///
/// Design note: We use f32 for on-device execution to save memory and battery.
/// Precision is not needed for motion segmentation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Monotonic timestamp in milliseconds. Required for temporal ordering.
    pub timestamp_ms: u64,

    /// Accelerometer reading [x, y, z] in m/s².
    pub accel: [f32; 3],

    /// Gyroscope reading [x, y, z] in rad/s.
    pub gyro: [f32; 3],
}

impl ImuSample {
    /// Creates a new IMU sample.
    ///
    /// Assumptions:
    /// - timestamp_ms must be monotonically increasing within a sequence
    /// - accel and gyro are calibrated (device-specific bias removed)
    pub fn new(timestamp_ms: u64, accel: [f32; 3], gyro: [f32; 3]) -> Self {
        Self {
            timestamp_ms,
            accel,
            gyro,
        }
    }

    /// Compute the magnitude of acceleration (useful for quick motion detection).
    /// Returns m/s².
    pub fn accel_magnitude(&self) -> f32 {
        let x2 = self.accel[0] * self.accel[0];
        let y2 = self.accel[1] * self.accel[1];
        let z2 = self.accel[2] * self.accel[2];
        (x2 + y2 + z2).sqrt()
    }

    /// Compute the magnitude of rotational velocity.
    /// Returns rad/s.
    pub fn gyro_magnitude(&self) -> f32 {
        let x2 = self.gyro[0] * self.gyro[0];
        let y2 = self.gyro[1] * self.gyro[1];
        let z2 = self.gyro[2] * self.gyro[2];
        (x2 + y2 + z2).sqrt()
    }
}

/// A window of time quantized into coarse duration buckets.
///
/// The engine intentionally avoids precise durations to enforce privacy
/// (prevents temporal fingerprinting) and reduce precision that would be
/// lost anyway during higher-level reasoning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DurationBucket {
    /// Less than 0.5 seconds.
    VeryShort,
    /// 0.5 to 2 seconds.
    Short,
    /// 2 to 5 seconds.
    Medium,
    /// 5 to 15 seconds.
    Long,
    /// 15+ seconds.
    VeryLong,
}

impl DurationBucket {
    /// Classify a duration (in milliseconds) into a bucket.
    pub fn from_ms(duration_ms: u64) -> Self {
        match duration_ms {
            0..=500 => DurationBucket::VeryShort,
            501..=2000 => DurationBucket::Short,
            2001..=5000 => DurationBucket::Medium,
            5001..=15000 => DurationBucket::Long,
            _ => DurationBucket::VeryLong,
        }
    }

    /// Returns the approximate center of the bucket (in milliseconds) for testing.
    pub fn representative_ms(&self) -> u64 {
        match self {
            DurationBucket::VeryShort => 250,
            DurationBucket::Short => 1000,
            DurationBucket::Medium => 3000,
            DurationBucket::Long => 10000,
            DurationBucket::VeryLong => 30000,
        }
    }
}

/// Health status of sensor data.
///
/// Signals whether the raw sensor data is in a trustworthy state or has degraded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorHealth {
    /// Nominal operation. All sensors are working as expected.
    Nominal,
    /// Degraded operation. Some signals are noisy or inconsistent,
    /// but processing can continue with reduced confidence.
    Degraded,
    /// Critical failure. Sensor data is unusable.
    Critical,
}

/// Validity state of a motion evidence window.
///
/// Every window must be explicitly marked with one of these states.
/// The system prefers incomplete evidence over incorrect inference.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidityState {
    /// Window meets all criteria for full confidence.
    Valid,
    /// Window is usable but with reduced confidence (e.g., sparse samples).
    Degraded,
    /// Window cannot be used. Invalid windows are dropped.
    Invalid,
}

/// A coarse contextual mode describing overall motion character.
///
/// This is intentionally high-level and does not constitute a prediction.
/// It merely summarizes the character of acceleration and rotation during
/// the window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionMode {
    /// Very low acceleration and rotation. Device is still.
    Still,
    /// Steady motion with minimal rotation. Likely walking or consistent translation.
    SteadyMotion,
    /// Elevated rotation without sustained translation. Likely turning in place.
    Turning,
    /// Sudden acceleration or deceleration. Likely a pause or transition event.
    Transitional,
}

/// A segment of motion with consistent character.
///
/// Segments are the result of temporal segmentation and represent continuous
/// stretches of similar motion behavior.
#[derive(Debug, Clone)]
pub struct MotionSegment {
    /// Timestamp (in ms) when this segment started.
    pub start_ms: u64,
    /// Duration of the segment (in ms).
    pub duration_ms: u64,
    /// Coarse mode of motion during this segment.
    pub mode: MotionMode,
    /// Average magnitude of acceleration during this segment (m/s²).
    pub avg_accel_mag: f32,
    /// Average magnitude of rotation during this segment (rad/s).
    pub avg_gyro_mag: f32,
    /// Confidence in this segment [0.0, 1.0].
    pub confidence: f32,
}

impl MotionSegment {
    /// Create a new motion segment.
    pub fn new(
        start_ms: u64,
        duration_ms: u64,
        mode: MotionMode,
        avg_accel_mag: f32,
        avg_gyro_mag: f32,
        confidence: f32,
    ) -> Self {
        Self {
            start_ms,
            duration_ms,
            mode,
            avg_accel_mag,
            avg_gyro_mag,
            confidence,
        }
    }

    /// Get the end timestamp of this segment.
    pub fn end_ms(&self) -> u64 {
        self.start_ms + self.duration_ms
    }

    /// Classify the duration of this segment into a bucket.
    pub fn duration_bucket(&self) -> DurationBucket {
        DurationBucket::from_ms(self.duration_ms)
    }
}

/// A motion transition event (e.g., stop, pause, turn).
///
/// Transitions are candidates for interruption events. They represent
/// boundaries or anomalies in the motion stream that warrant attention.
#[derive(Debug, Clone)]
pub struct TransitionCandidate {
    /// Timestamp (in ms) when this transition was detected.
    pub timestamp_ms: u64,
    /// Type of transition.
    pub transition_type: TransitionType,
    /// Confidence in this detection [0.0, 1.0].
    pub confidence: f32,
    /// Duration of the transition event (in ms).
    pub duration_ms: u64,
}

impl TransitionCandidate {
    /// Create a new transition candidate.
    pub fn new(
        timestamp_ms: u64,
        transition_type: TransitionType,
        confidence: f32,
        duration_ms: u64,
    ) -> Self {
        Self {
            timestamp_ms,
            transition_type,
            confidence,
            duration_ms,
        }
    }

    /// Classify the duration of this transition.
    pub fn duration_bucket(&self) -> DurationBucket {
        DurationBucket::from_ms(self.duration_ms)
    }
}

/// Classification of transition types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionType {
    /// Sudden deceleration without reversing. Possible moment of interruption.
    AbruptStop,
    /// Sustained pause (duration > 500ms) after motion. Likely placement event.
    Pause,
    /// Sudden change in heading direction (>45 degrees).
    DirectionChange,
    /// Brief hesitation (micro-pause) during motion.
    Hesitation,
}

/// A complete motion evidence window.
///
/// This is the primary output of the IMU Movement Engine. It represents
/// a window of time (typically a few seconds) analyzed for motion evidence.
///
/// Design note: This structure is intentionally evidence-only. It contains
/// no interpretation, no location data, no identity markers. Higher layers
/// must add semantic meaning.
#[derive(Debug, Clone)]
pub struct MotionEvidenceWindow {
    /// Unique identifier for this window (for tracking and versioning).
    pub window_id: u64,
    /// Timestamp (in ms) when this window starts.
    pub start_ms: u64,
    /// Timestamp (in ms) when this window ends.
    pub end_ms: u64,
    /// Segments of consistent motion found within this window.
    pub segments: Vec<MotionSegment>,
    /// Transition candidates found within this window.
    pub transitions: Vec<TransitionCandidate>,
    /// Coarse contextual mode of this window.
    pub context_mode: MotionMode,
    /// Confidence in this window's validity [0.0, 1.0].
    pub confidence: f32,
    /// Health of the sensor data during this window.
    pub sensor_health: SensorHealth,
    /// Validity state of this window.
    pub validity_state: ValidityState,
    /// Schema version for versioning and compatibility.
    pub schema_version: u32,
}

impl MotionEvidenceWindow {
    /// Create a new motion evidence window.
    pub fn new(
        window_id: u64,
        start_ms: u64,
        end_ms: u64,
        context_mode: MotionMode,
        sensor_health: SensorHealth,
        schema_version: u32,
    ) -> Self {
        Self {
            window_id,
            start_ms,
            end_ms,
            segments: Vec::new(),
            transitions: Vec::new(),
            context_mode,
            confidence: 1.0,
            sensor_health,
            validity_state: ValidityState::Valid,
            schema_version,
        }
    }

    /// Get the duration of this window in milliseconds.
    pub fn duration_ms(&self) -> u64 {
        self.end_ms - self.start_ms
    }

    /// Classify the duration of this window into a bucket.
    pub fn duration_bucket(&self) -> DurationBucket {
        DurationBucket::from_ms(self.duration_ms())
    }

    /// Add a motion segment to this window.
    pub fn add_segment(&mut self, segment: MotionSegment) {
        self.segments.push(segment);
    }

    /// Add a transition candidate to this window.
    pub fn add_transition(&mut self, transition: TransitionCandidate) {
        self.transitions.push(transition);
    }

    /// Mark this window as invalid.
    pub fn mark_invalid(&mut self) {
        self.validity_state = ValidityState::Invalid;
    }

    /// Mark this window as degraded with reduced confidence.
    pub fn mark_degraded(&mut self, confidence: f32) {
        self.validity_state = ValidityState::Degraded;
        self.confidence = confidence.clamp(0.0, 1.0);
    }

    /// Check if this window has any transition candidates.
    pub fn has_transitions(&self) -> bool {
        !self.transitions.is_empty()
    }

    /// Check if this window is valid for downstream processing.
    pub fn is_valid(&self) -> bool {
        self.validity_state == ValidityState::Valid
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_sample_magnitudes() {
        let sample = ImuSample::new(0, [3.0, 4.0, 0.0], [1.0, 0.0, 0.0]);
        assert_eq!(sample.accel_magnitude(), 5.0);
        assert_eq!(sample.gyro_magnitude(), 1.0);
    }

    #[test]
    fn test_duration_bucket_boundaries() {
        assert_eq!(DurationBucket::from_ms(100), DurationBucket::VeryShort);
        assert_eq!(DurationBucket::from_ms(500), DurationBucket::VeryShort);
        assert_eq!(DurationBucket::from_ms(501), DurationBucket::Short);
        assert_eq!(DurationBucket::from_ms(2000), DurationBucket::Short);
        assert_eq!(DurationBucket::from_ms(2001), DurationBucket::Medium);
        assert_eq!(DurationBucket::from_ms(5000), DurationBucket::Medium);
        assert_eq!(DurationBucket::from_ms(5001), DurationBucket::Long);
        assert_eq!(DurationBucket::from_ms(15000), DurationBucket::Long);
        assert_eq!(DurationBucket::from_ms(15001), DurationBucket::VeryLong);
    }

    #[test]
    fn test_motion_evidence_window_creation() {
        let window = MotionEvidenceWindow::new(
            1,
            0,
            5000,
            MotionMode::SteadyMotion,
            SensorHealth::Nominal,
            1,
        );

        assert_eq!(window.duration_ms(), 5000);
        assert_eq!(window.duration_bucket(), DurationBucket::Medium);
        assert!(window.is_valid());
        assert!(!window.has_transitions());
    }

    #[test]
    fn test_window_validity_states() {
        let mut window = MotionEvidenceWindow::new(
            1,
            0,
            1000,
            MotionMode::Still,
            SensorHealth::Nominal,
            1,
        );

        assert_eq!(window.validity_state, ValidityState::Valid);

        window.mark_degraded(0.75);
        assert_eq!(window.validity_state, ValidityState::Degraded);
        assert_eq!(window.confidence, 0.75);

        window.mark_invalid();
        assert_eq!(window.validity_state, ValidityState::Invalid);
        assert!(!window.is_valid());
    }

    #[test]
    fn test_segment_end_time() {
        let segment = MotionSegment::new(1000, 2000, MotionMode::SteadyMotion, 5.0, 0.1, 0.9);
        assert_eq!(segment.end_ms(), 3000);
    }

    #[test]
    fn test_transition_candidate_classification() {
        let trans = TransitionCandidate::new(1000, TransitionType::Pause, 0.85, 800);
        assert_eq!(trans.duration_bucket(), DurationBucket::Short);
    }

    #[test]
    fn test_confidence_clamping() {
        let mut window = MotionEvidenceWindow::new(
            1,
            0,
            1000,
            MotionMode::Still,
            SensorHealth::Nominal,
            1,
        );

        window.mark_degraded(2.0); // Out of range
        assert_eq!(window.confidence, 1.0); // Clamped to max

        window.mark_degraded(-0.5); // Out of range
        assert_eq!(window.confidence, 0.0); // Clamped to min
    }
}
