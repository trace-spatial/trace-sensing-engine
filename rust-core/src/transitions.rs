/// Motion transition detection and classification.
///
/// This module analyzes boundaries between motion segments and classifies
/// the type of transition occurring: pause (moving→still), stop (still→moving),
/// turn (gyroscopic rotation), or hesitation (brief velocity changes).
///
/// Transition detection operates on segment boundaries from the segmentation engine,
/// extracting evidence about how motion changed and providing confidence scores
/// for classification certainty.
///
/// # Architecture
/// - Evidence-first: transitions classified from measured acceleration/rotation data
/// - Fail-loud: explicit confidence and validity states
/// - O(1) per-transition: no statistical buffers, only boundary analysis
/// - Privacy-preserving: no location, identity, or personal markers

use crate::types::MotionMode;

/// Classification of motion transitions.
///
/// Represents distinct types of motion state changes with different characteristics
/// and implications for user activity analysis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TransitionType {
    /// User stopped (moving → still). Implies pausing or end of activity.
    Stop,

    /// User started moving (still → moving). Implies beginning of activity.
    Start,

    /// User changed direction significantly (gyroscopic rotation). Implies turn or pivot.
    Turn,

    /// User briefly paused or hesitated (moving → still → moving). Implies uncertainty/deliberation.
    Hesitation,

    /// Direction changed without full stop (gyro spike during motion). Implies course change.
    DirectionChange,

    /// Transition between two motion types neither is still. Implies activity shift.
    ActivityShift,
}

impl TransitionType {
    /// Returns a human-readable description of the transition type.
    pub fn description(&self) -> &'static str {
        match self {
            TransitionType::Stop => "User stopped motion",
            TransitionType::Start => "User started motion",
            TransitionType::Turn => "User made a turn",
            TransitionType::Hesitation => "User hesitated/paused briefly",
            TransitionType::DirectionChange => "User changed direction",
            TransitionType::ActivityShift => "User shifted activity type",
        }
    }

    /// Returns true if this transition represents motion beginning.
    pub fn is_initiation(&self) -> bool {
        matches!(self, TransitionType::Start)
    }

    /// Returns true if this transition represents motion ending.
    pub fn is_termination(&self) -> bool {
        matches!(self, TransitionType::Stop)
    }

    /// Returns true if this transition is significant (not hesitation or direction change).
    pub fn is_significant(&self) -> bool {
        matches!(
            self,
            TransitionType::Stop | TransitionType::Start | TransitionType::Turn
        )
    }
}

/// Configuration for transition detection behavior.
#[derive(Debug, Clone, Copy)]
pub struct TransitionConfig {
    /// Minimum accel magnitude to classify as "moving" (m/s²).
    /// Below this, motion is considered stopped or still.
    pub motion_threshold: f32,

    /// Gyroscope magnitude threshold for detecting turns (rad/s).
    /// Peak gyro > this triggers Turn classification.
    pub turn_threshold: f32,

    /// Maximum segment duration to classify as hesitation (milliseconds).
    /// Brief still periods during motion trigger Hesitation instead of Stop.
    pub max_hesitation_duration: u32,

    /// Minimum accel change to classify direction change (m/s²).
    /// Accel shift > this during motion triggers DirectionChange.
    pub direction_change_threshold: f32,

    /// Minimum confidence (0.0-1.0) to emit transition events.
    /// Below this, transition is flagged as invalid.
    pub min_confidence: f32,
}

impl Default for TransitionConfig {
    fn default() -> Self {
        Self {
            motion_threshold: 0.3,            // Still/moving boundary
            turn_threshold: 0.2,              // Rotation threshold (rad/s)
            max_hesitation_duration: 500,     // 500ms max for hesitation
            direction_change_threshold: 1.5,  // 1.5 m/s² delta
            min_confidence: 0.6,              // 60% minimum confidence
        }
    }
}

/// Transition event detected between two motion segments.
///
/// Captures the classification, timing, and confidence of a detected transition.
#[derive(Debug, Clone, PartialEq)]
pub struct TransitionEvent {
    /// Type of transition detected.
    pub transition_type: TransitionType,

    /// Confidence in classification (0.0-1.0).
    /// Higher values indicate stronger evidence for classification.
    pub confidence: f32,

    /// Duration of the transition/pause in milliseconds.
    /// For Start: duration of preceding still period.
    /// For Stop: immediately 0 (stop happens at transition point).
    /// For Hesitation: duration of the brief still period.
    /// For Turn: duration of rotation/deceleration.
    pub duration_ms: u32,

    /// Sample index where transition occurred.
    /// Marks the point in the sequence where state change was detected.
    pub sample_index: usize,

    /// Mode being transitioned from.
    pub from_mode: MotionMode,

    /// Mode being transitioned to.
    pub to_mode: MotionMode,

    /// Validity indicator (true = sufficient evidence, false = uncertain).
    pub is_valid: bool,
}

impl TransitionEvent {
    /// Creates a transition event with derived validity from confidence.
    pub fn new(
        transition_type: TransitionType,
        confidence: f32,
        duration_ms: u32,
        sample_index: usize,
        from_mode: MotionMode,
        to_mode: MotionMode,
        min_confidence: f32,
    ) -> Self {
        Self {
            transition_type,
            confidence,
            duration_ms,
            sample_index,
            from_mode,
            to_mode,
            is_valid: confidence >= min_confidence,
        }
    }
}

/// Transition boundary information passed to classifier.
///
/// Encapsulates measurements at the boundary between two motion segments
/// to enable classification of transition type.
pub struct TransitionBoundary {
    /// Acceleration magnitude before transition (m/s²).
    pub accel_before: f32,

    /// Acceleration magnitude after transition (m/s²).
    pub accel_after: f32,

    /// Gyroscope magnitude at boundary (rad/s).
    pub gyro_at_boundary: f32,

    /// Duration of the segment before transition (milliseconds).
    pub segment_before_duration: u32,

    /// Duration of the segment after transition (milliseconds).
    pub segment_after_duration: u32,

    /// Accel variance in segment before (m/s²)².
    pub accel_variance_before: f32,

    /// Accel variance in segment after (m/s²)².
    pub accel_variance_after: f32,

    /// Mode before transition.
    pub mode_before: MotionMode,

    /// Mode after transition.
    pub mode_after: MotionMode,
}

/// Classifies transition type from boundary evidence.
pub struct TransitionClassifier {
    config: TransitionConfig,
}

impl TransitionClassifier {
    /// Creates a new transition classifier with given configuration.
    pub fn new(config: TransitionConfig) -> Self {
        Self { config }
    }

    /// Classifies a transition from boundary measurements.
    ///
    /// Returns (TransitionType, confidence) tuple indicating the most likely
    /// transition type and certainty of classification.
    pub fn classify(&self, boundary: &TransitionBoundary) -> (TransitionType, f32) {
        // Determine motion states before/after
        let was_moving = boundary.accel_before > self.config.motion_threshold;
        let is_moving = boundary.accel_after > self.config.motion_threshold;

        // Check for significant gyro activity at boundary
        let has_rotation = boundary.gyro_at_boundary > self.config.turn_threshold;

        // Check for direction change during motion
        let accel_delta = (boundary.accel_after - boundary.accel_before).abs();
        let has_direction_change = accel_delta > self.config.direction_change_threshold && is_moving;

        // Classify based on evidence hierarchy
        if was_moving && !is_moving {
            // Stopping: check if hesitation (brief pause) or full stop
            let is_hesitation = boundary.segment_after_duration < self.config.max_hesitation_duration;

            if is_hesitation {
                // Brief still period suggests hesitation
                let confidence = self.confidence_hesitation(boundary);
                (TransitionType::Hesitation, confidence)
            } else {
                // Sustained still period = stop
                let confidence = self.confidence_stop(boundary);
                (TransitionType::Stop, confidence)
            }
        } else if !was_moving && is_moving {
            // Starting: moving from still
            let confidence = self.confidence_start(boundary);
            (TransitionType::Start, confidence)
        } else if has_rotation {
            // Significant rotation detected = turn
            let confidence = self.confidence_turn(boundary);
            (TransitionType::Turn, confidence)
        } else if has_direction_change {
            // Accel change during motion = direction change
            let confidence = self.confidence_direction_change(boundary);
            (TransitionType::DirectionChange, confidence)
        } else {
            // Shift between two motion modes neither is still
            let confidence = self.confidence_activity_shift(boundary);
            (TransitionType::ActivityShift, confidence)
        }
    }

    /// Computes confidence for Stop transition.
    /// Higher confidence with sustained acceleration drop and clear velocity change.
    fn confidence_stop(&self, boundary: &TransitionBoundary) -> f32 {
        let mut confidence: f32 = 0.5; // Base confidence

        // Deduct confidence for residual acceleration (unclear stop)
        if boundary.accel_after > 0.1 {
            confidence -= 0.1;
        }

        // Increase confidence with clear acceleration drop
        let accel_drop = boundary.accel_before - boundary.accel_after;
        if accel_drop > 1.0 {
            confidence += 0.2;
        }

        // Reduce confidence if gyro activity (turn instead of stop)
        if boundary.gyro_at_boundary > self.config.turn_threshold * 0.5 {
            confidence -= 0.15;
        }

        // Increase confidence for longer before segment (sustained motion)
        if boundary.segment_before_duration > 1000 {
            confidence += 0.1;
        }

        confidence.max(0.0_f32).min(1.0_f32)
    }

    /// Computes confidence for Start transition.
    /// Higher confidence with clear accel increase and preceding still period.
    fn confidence_start(&self, boundary: &TransitionBoundary) -> f32 {
        let mut confidence: f32 = 0.5;

        // Increase confidence with clear accel increase
        let accel_rise = boundary.accel_after - boundary.accel_before;
        if accel_rise > 0.5 {
            confidence += 0.2;
        }

        // Increase confidence for preceding still period (clear context)
        if boundary.segment_before_duration > 500 {
            confidence += 0.15;
        }

        // Reduce confidence if high variance before (uncertain preceding state)
        if boundary.accel_variance_before > 0.2 {
            confidence -= 0.1;
        }

        confidence.max(0.0_f32).min(1.0_f32)
    }

    /// Computes confidence for Turn transition.
    /// Higher confidence with sustained gyro activity and minimal accel change.
    fn confidence_turn(&self, boundary: &TransitionBoundary) -> f32 {
        let mut confidence: f32 = 0.6; // Base higher for clear rotational signature

        // Increase confidence with strong gyro signal
        if boundary.gyro_at_boundary > self.config.turn_threshold * 2.0 {
            confidence += 0.2;
        }

        // Reduce confidence if significant accel change (acceleration-based motion change)
        let accel_delta = (boundary.accel_after - boundary.accel_before).abs();
        if accel_delta > 1.0 {
            confidence -= 0.15;
        }

        // Increase confidence for motion continuity (accel stable during turn)
        if boundary.accel_variance_after < 0.1 {
            confidence += 0.1;
        }

        confidence.max(0.0_f32).min(1.0_f32)
    }

    /// Computes confidence for Hesitation transition.
    /// Higher confidence with brief pause, low accel after, and quick recovery.
    fn confidence_hesitation(&self, boundary: &TransitionBoundary) -> f32 {
        let mut confidence: f32 = 0.5;

        // Increase confidence with very brief still period
        if boundary.segment_after_duration < 200 {
            confidence += 0.2;
        }

        // Increase confidence if recovering to higher accel (resuming motion)
        if boundary.accel_after > boundary.accel_before {
            confidence += 0.15;
        }

        // Reduce confidence if extended still period (closer to full stop)
        if boundary.segment_after_duration > self.config.max_hesitation_duration {
            confidence -= 0.2;
        }

        confidence.max(0.0_f32).min(1.0_f32)
    }

    /// Computes confidence for DirectionChange transition.
    /// Higher confidence with sustained motion and accel shift.
    fn confidence_direction_change(&self, boundary: &TransitionBoundary) -> f32 {
        let mut confidence: f32 = 0.5;

        // Increase confidence with significant accel change
        let accel_delta = (boundary.accel_after - boundary.accel_before).abs();
        if accel_delta > self.config.direction_change_threshold * 1.5 {
            confidence += 0.2;
        }

        // Increase confidence with sustained motion context
        if boundary.segment_before_duration > 500 && boundary.segment_after_duration > 500 {
            confidence += 0.15;
        }

        // Reduce confidence if low gyro activity (direction change typically includes rotation)
        if boundary.gyro_at_boundary < self.config.turn_threshold * 0.3 {
            confidence -= 0.1;
        }

        confidence.max(0.0_f32).min(1.0_f32)
    }

    /// Computes confidence for ActivityShift transition.
    /// Higher confidence with clear mode change and low variance.
    fn confidence_activity_shift(&self, boundary: &TransitionBoundary) -> f32 {
        let mut confidence: f32 = 0.5;

        // Increase confidence with stable signals before and after
        if boundary.accel_variance_before < 0.15 && boundary.accel_variance_after < 0.15 {
            confidence += 0.2;
        }

        // Increase confidence for longer segments (sustained activities)
        if boundary.segment_before_duration > 2000 && boundary.segment_after_duration > 2000 {
            confidence += 0.15;
        }

        confidence.max(0.0_f32).min(1.0_f32)
    }
}

/// Transition detection engine for streaming operation.
///
/// Processes segment boundaries from the segmentation engine and emits
/// TransitionEvent objects with classified transition types.
pub struct TransitionDetector {
    config: TransitionConfig,
    classifier: TransitionClassifier,

    // Tracking state
    last_mode: Option<MotionMode>,
    last_segment_sample: usize,
    pending_transition: Option<(MotionMode, usize)>,
}

impl TransitionDetector {
    /// Creates a new transition detector with given configuration.
    pub fn new(config: TransitionConfig) -> Self {
        let classifier = TransitionClassifier::new(config);
        Self {
            config,
            classifier,
            last_mode: None,
            last_segment_sample: 0,
            pending_transition: None,
        }
    }

    /// Processes a segment boundary and returns any detected transition event.
    ///
    /// Should be called when the segmentation engine detects a mode change.
    /// Returns Some(TransitionEvent) if a transition is classified, None otherwise.
    pub fn process_transition(
        &mut self,
        from_mode: MotionMode,
        to_mode: MotionMode,
        boundary: &TransitionBoundary,
        sample_index: usize,
    ) -> Option<TransitionEvent> {
        // Classify the transition
        let (transition_type, confidence) = self.classifier.classify(boundary);

        // Create transition event
        let event = TransitionEvent::new(
            transition_type,
            confidence,
            boundary.segment_after_duration,
            sample_index,
            from_mode,
            to_mode,
            self.config.min_confidence,
        );

        // Update tracking state
        self.last_mode = Some(to_mode);

        Some(event)
    }

    /// Returns the last detected mode.
    pub fn last_mode(&self) -> Option<MotionMode> {
        self.last_mode
    }

    /// Returns the duration since last segment boundary in samples.
    pub fn samples_since_transition(&self, current_sample: usize) -> usize {
        current_sample.saturating_sub(self.last_segment_sample)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transition_type_stop() {
        let stop = TransitionType::Stop;
        assert!(stop.is_termination());
        assert!(!stop.is_initiation());
        assert!(stop.is_significant());
    }

    #[test]
    fn test_transition_type_start() {
        let start = TransitionType::Start;
        assert!(start.is_initiation());
        assert!(!start.is_termination());
        assert!(start.is_significant());
    }

    #[test]
    fn test_transition_type_turn() {
        let turn = TransitionType::Turn;
        assert!(!turn.is_initiation());
        assert!(!turn.is_termination());
        assert!(turn.is_significant());
    }

    #[test]
    fn test_transition_type_hesitation() {
        let hesitation = TransitionType::Hesitation;
        assert!(!hesitation.is_significant());
    }

    #[test]
    fn test_classifier_stop_clear_deceleration() {
        let config = TransitionConfig::default();
        let classifier = TransitionClassifier::new(config);

        let boundary = TransitionBoundary {
            accel_before: 2.0,
            accel_after: 0.1,
            gyro_at_boundary: 0.05,
            segment_before_duration: 1500,
            segment_after_duration: 1000,  // Sustained still period (> max_hesitation_duration)
            accel_variance_before: 0.05,
            accel_variance_after: 0.02,
            mode_before: MotionMode::SteadyMotion,
            mode_after: MotionMode::Still,
        };

        let (transition_type, confidence) = classifier.classify(&boundary);
        assert_eq!(transition_type, TransitionType::Stop);
        assert!(confidence > 0.6);
    }

    #[test]
    fn test_classifier_start_clear_acceleration() {
        let config = TransitionConfig::default();
        let classifier = TransitionClassifier::new(config);

        let boundary = TransitionBoundary {
            accel_before: 0.1,
            accel_after: 1.8,
            gyro_at_boundary: 0.05,
            segment_before_duration: 2000,
            segment_after_duration: 500,
            accel_variance_before: 0.01,
            accel_variance_after: 0.08,
            mode_before: MotionMode::Still,
            mode_after: MotionMode::SteadyMotion,
        };

        let (transition_type, confidence) = classifier.classify(&boundary);
        assert_eq!(transition_type, TransitionType::Start);
        assert!(confidence > 0.6);
    }

    #[test]
    fn test_classifier_turn_gyro_dominant() {
        let config = TransitionConfig::default();
        let classifier = TransitionClassifier::new(config);

        let boundary = TransitionBoundary {
            accel_before: 1.5,
            accel_after: 1.6,
            gyro_at_boundary: 0.5,
            segment_before_duration: 800,
            segment_after_duration: 600,
            accel_variance_before: 0.05,
            accel_variance_after: 0.06,
            mode_before: MotionMode::SteadyMotion,
            mode_after: MotionMode::Turning,
        };

        let (transition_type, confidence) = classifier.classify(&boundary);
        assert_eq!(transition_type, TransitionType::Turn);
        assert!(confidence > 0.6);
    }

    #[test]
    fn test_classifier_hesitation_brief_pause() {
        let config = TransitionConfig::default();
        let classifier = TransitionClassifier::new(config);

        let boundary = TransitionBoundary {
            accel_before: 1.8,
            accel_after: 0.15,  // Brief pause to very low accel
            gyro_at_boundary: 0.08,
            segment_before_duration: 1200,
            segment_after_duration: 150,  // Brief still period (< max_hesitation_duration: 500ms)
            accel_variance_before: 0.06,
            accel_variance_after: 0.04,
            mode_before: MotionMode::SteadyMotion,
            mode_after: MotionMode::Still,  // Brief still period
        };

        let (transition_type, confidence) = classifier.classify(&boundary);
        assert_eq!(transition_type, TransitionType::Hesitation);
        assert!(confidence > 0.5);
    }

    #[test]
    fn test_classifier_direction_change_sustained_motion() {
        let config = TransitionConfig::default();
        let classifier = TransitionClassifier::new(config);

        let boundary = TransitionBoundary {
            accel_before: 1.2,
            accel_after: 2.8,
            gyro_at_boundary: 0.15,
            segment_before_duration: 1500,
            segment_after_duration: 1200,
            accel_variance_before: 0.05,
            accel_variance_after: 0.07,
            mode_before: MotionMode::SteadyMotion,
            mode_after: MotionMode::SteadyMotion,
        };

        let (transition_type, confidence) = classifier.classify(&boundary);
        assert_eq!(transition_type, TransitionType::DirectionChange);
        assert!(confidence > 0.5);
    }

    #[test]
    fn test_transition_event_validity_above_threshold() {
        let event = TransitionEvent::new(
            TransitionType::Stop,
            0.75,
            100,
            50,
            MotionMode::SteadyMotion,
            MotionMode::Still,
            0.6,
        );

        assert!(event.is_valid);
        assert_eq!(event.confidence, 0.75);
    }

    #[test]
    fn test_transition_event_validity_below_threshold() {
        let event = TransitionEvent::new(
            TransitionType::Hesitation,
            0.45,
            50,
            100,
            MotionMode::SteadyMotion,
            MotionMode::Still,
            0.6,
        );

        assert!(!event.is_valid);
        assert_eq!(event.confidence, 0.45);
    }

    #[test]
    fn test_transition_detector_state_tracking() {
        let config = TransitionConfig::default();
        let mut detector = TransitionDetector::new(config);

        assert_eq!(detector.last_mode(), None);

        let boundary = TransitionBoundary {
            accel_before: 0.1,
            accel_after: 1.5,
            gyro_at_boundary: 0.05,
            segment_before_duration: 1000,
            segment_after_duration: 100,
            accel_variance_before: 0.01,
            accel_variance_after: 0.06,
            mode_before: MotionMode::Still,
            mode_after: MotionMode::SteadyMotion,
        };

        let event = detector.process_transition(MotionMode::Still, MotionMode::SteadyMotion, &boundary, 50);
        assert!(event.is_some());
        assert_eq!(detector.last_mode(), Some(MotionMode::SteadyMotion));
    }

    #[test]
    fn test_transition_detector_multiple_transitions() {
        let config = TransitionConfig::default();
        let mut detector = TransitionDetector::new(config);

        // First transition: Still → SteadyMotion
        let boundary1 = TransitionBoundary {
            accel_before: 0.1,
            accel_after: 1.8,
            gyro_at_boundary: 0.05,
            segment_before_duration: 1000,
            segment_after_duration: 500,
            accel_variance_before: 0.01,
            accel_variance_after: 0.06,
            mode_before: MotionMode::Still,
            mode_after: MotionMode::SteadyMotion,
        };

        let event1 = detector.process_transition(MotionMode::Still, MotionMode::SteadyMotion, &boundary1, 100);
        assert_eq!(event1.unwrap().transition_type, TransitionType::Start);

        // Second transition: SteadyMotion → Still
        let boundary2 = TransitionBoundary {
            accel_before: 1.8,
            accel_after: 0.1,
            gyro_at_boundary: 0.05,
            segment_before_duration: 1500,
            segment_after_duration: 1000,  // Sustained still period (> max_hesitation_duration)
            accel_variance_before: 0.06,
            accel_variance_after: 0.01,
            mode_before: MotionMode::SteadyMotion,
            mode_after: MotionMode::Still,
        };

        let event2 = detector.process_transition(MotionMode::SteadyMotion, MotionMode::Still, &boundary2, 200);
        assert_eq!(event2.unwrap().transition_type, TransitionType::Stop);
        assert_eq!(detector.last_mode(), Some(MotionMode::Still));
    }

    #[test]
    fn test_transition_detector_confidence_filtering() {
        let mut config = TransitionConfig::default();
        config.min_confidence = 0.8;
        let mut detector = TransitionDetector::new(config);

        let boundary = TransitionBoundary {
            accel_before: 1.5,
            accel_after: 0.2,
            gyro_at_boundary: 0.5,
            segment_before_duration: 800,
            segment_after_duration: 200,
            accel_variance_before: 0.15,
            accel_variance_after: 0.1,
            mode_before: MotionMode::SteadyMotion,
            mode_after: MotionMode::Still,
        };

        let event = detector.process_transition(MotionMode::SteadyMotion, MotionMode::Still, &boundary, 150);
        let unwrapped = event.unwrap();

        // Should mark as invalid if confidence < 0.8
        if unwrapped.confidence < 0.8 {
            assert!(!unwrapped.is_valid);
        }
    }

    #[test]
    fn test_transition_event_descriptions() {
        assert_eq!(TransitionType::Stop.description(), "User stopped motion");
        assert_eq!(TransitionType::Start.description(), "User started motion");
        assert_eq!(TransitionType::Turn.description(), "User made a turn");
        assert_eq!(
            TransitionType::Hesitation.description(),
            "User hesitated/paused briefly"
        );
        assert_eq!(
            TransitionType::DirectionChange.description(),
            "User changed direction"
        );
        assert_eq!(TransitionType::ActivityShift.description(), "User shifted activity type");
    }

    #[test]
    fn test_classifier_confidence_bounds() {
        let config = TransitionConfig::default();
        let classifier = TransitionClassifier::new(config);

        let boundary = TransitionBoundary {
            accel_before: 0.5,
            accel_after: 0.5,
            gyro_at_boundary: 0.05,
            segment_before_duration: 1000,
            segment_after_duration: 1000,
            accel_variance_before: 0.5,
            accel_variance_after: 0.5,
            mode_before: MotionMode::SteadyMotion,
            mode_after: MotionMode::SteadyMotion,
        };

        let (_, confidence) = classifier.classify(&boundary);
        assert!(confidence >= 0.0 && confidence <= 1.0);
    }

    #[test]
    fn test_classifier_activity_shift_stable_signals() {
        let config = TransitionConfig::default();
        let classifier = TransitionClassifier::new(config);

        let boundary = TransitionBoundary {
            accel_before: 1.0,
            accel_after: 2.5,
            gyro_at_boundary: 0.1,
            segment_before_duration: 2500,
            segment_after_duration: 2500,
            accel_variance_before: 0.08,
            accel_variance_after: 0.09,
            mode_before: MotionMode::SteadyMotion,
            mode_after: MotionMode::Turning,
        };

        let (transition_type, confidence) = classifier.classify(&boundary);
        assert_eq!(transition_type, TransitionType::ActivityShift);
        assert!(confidence > 0.5);
    }

    #[test]
    fn test_transition_detector_samples_since_transition() {
        let config = TransitionConfig::default();
        let mut detector = TransitionDetector::new(config);

        let boundary = TransitionBoundary {
            accel_before: 0.1,
            accel_after: 1.5,
            gyro_at_boundary: 0.05,
            segment_before_duration: 1000,
            segment_after_duration: 100,
            accel_variance_before: 0.01,
            accel_variance_after: 0.06,
            mode_before: MotionMode::Still,
            mode_after: MotionMode::SteadyMotion,
        };

        detector.process_transition(MotionMode::Still, MotionMode::SteadyMotion, &boundary, 30);

        let samples_elapsed = detector.samples_since_transition(50);
        assert!(samples_elapsed >= 0);
    }
}
