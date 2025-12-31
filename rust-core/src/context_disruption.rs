//! Context Disruption Detection Module
//!
//! This is the CORE of what makes Trace valuable. It detects moments when
//! a user's attention or context may have been disrupted - the exact moments
//! when someone might "lose" their mental thread of what they were doing.
//!
//! Context disruptions include:
//! - Sudden stops after sustained motion (interrupted journey)
//! - Phone placement events (put phone down = attention shifted)
//! - Significant heading changes (disorientation)
//! - Transport mode transitions (getting off bus = new context)
//! - Hesitation patterns (pause-resume = uncertainty)
//! - Multi-tasking signatures (repeated short movements)
//!
//! The output feeds into CEBE-X for ML-based context reconstruction.

use crate::types::*;

/// Types of context disruption events.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisruptionType {
    /// Phone placed down - strong signal of attention shift
    PhonePlacement,
    /// Sudden stop after sustained walking - interrupted journey
    AbruptHalt,
    /// Significant direction change - possible disorientation
    DirectionChange,
    /// Transport mode changed - environmental context shift
    TransportTransition,
    /// Brief hesitation during motion - decision point
    Hesitation,
    /// Phone picked up after placement - context resumption attempt
    ContextResumption,
    /// Unusual stillness duration - prolonged distraction
    ExtendedStillness,
    /// Rapid repeated movements - multi-tasking / searching behavior
    SearchingBehavior,
    /// Entering/exiting building (elevator, stairs)
    EnvironmentTransition,
}

impl DisruptionType {
    /// Get the base importance weight for this disruption type.
    /// Higher = more likely to be a significant context loss moment.
    pub fn base_weight(&self) -> f32 {
        match self {
            DisruptionType::PhonePlacement => 0.9,      // Very strong signal
            DisruptionType::AbruptHalt => 0.7,
            DisruptionType::DirectionChange => 0.4,
            DisruptionType::TransportTransition => 0.8,
            DisruptionType::Hesitation => 0.5,
            DisruptionType::ContextResumption => 0.6,
            DisruptionType::ExtendedStillness => 0.6,
            DisruptionType::SearchingBehavior => 0.75,
            DisruptionType::EnvironmentTransition => 0.65,
        }
    }

    /// Human-readable description for CEBE-X.
    pub fn description(&self) -> &'static str {
        match self {
            DisruptionType::PhonePlacement => "phone_placed_down",
            DisruptionType::AbruptHalt => "sudden_stop",
            DisruptionType::DirectionChange => "direction_change",
            DisruptionType::TransportTransition => "transport_changed",
            DisruptionType::Hesitation => "hesitation",
            DisruptionType::ContextResumption => "context_resumed",
            DisruptionType::ExtendedStillness => "extended_still",
            DisruptionType::SearchingBehavior => "searching_behavior",
            DisruptionType::EnvironmentTransition => "environment_transition",
        }
    }
}

/// A detected context disruption event.
#[derive(Debug, Clone)]
pub struct ContextDisruption {
    /// Timestamp when disruption occurred (ms).
    pub timestamp_ms: u64,
    /// Type of disruption.
    pub disruption_type: DisruptionType,
    /// Confidence in this detection [0.0, 1.0].
    pub confidence: f32,
    /// Importance score for ranking [0.0, 1.0].
    /// Combines type weight + contextual factors.
    pub importance: f32,
    /// Position at disruption (from PDR).
    pub position: TrajectoryPoint,
    /// Duration of the disruption event (ms).
    pub duration_ms: u64,
    /// Contextual features for CEBE-X.
    pub features: DisruptionFeatures,
}

/// Rich features extracted at disruption moment for CEBE-X ML processing.
#[derive(Debug, Clone, Default)]
pub struct DisruptionFeatures {
    // Motion context before disruption
    /// Seconds of motion before this disruption.
    pub motion_duration_before_s: f32,
    /// Average speed before disruption (m/s).
    pub avg_speed_before_mps: f32,
    /// Steps taken in last 30 seconds.
    pub steps_last_30s: u32,
    /// Distance traveled in last 30 seconds (m).
    pub distance_last_30s_m: f32,
    
    // Heading/orientation context
    /// Heading change in last 10 seconds (radians).
    pub heading_change_10s_rad: f32,
    /// Number of turns in last 30 seconds.
    pub turns_last_30s: u32,
    
    // Stillness context
    /// Seconds still before disruption (0 if was moving).
    pub stillness_before_s: f32,
    /// Seconds still after disruption.
    pub stillness_after_s: f32,
    
    // Phone interaction context
    /// Was phone picked up recently (last 30s)?
    pub recent_pickup: bool,
    /// Was phone placed down recently (last 30s)?
    pub recent_putdown: bool,
    /// Number of phone interactions in last 60s.
    pub phone_interactions_60s: u32,
    
    // Transport context
    /// Transport mode at disruption.
    pub transport_mode: TransportMode,
    /// Was there a recent transport change?
    pub recent_transport_change: bool,
    
    // Time context (for CEBE-X patterns)
    /// Milliseconds since last disruption.
    pub ms_since_last_disruption: u64,
    /// Total disruptions in current session.
    pub session_disruption_count: u32,
}

/// Configuration for context disruption detection.
#[derive(Debug, Clone)]
pub struct DisruptionConfig {
    // Timing thresholds
    /// Minimum motion duration to consider abrupt halt (ms).
    pub min_motion_for_halt_ms: u64,
    /// Stillness duration to trigger extended stillness (ms).
    pub extended_stillness_threshold_ms: u64,
    /// Hesitation detection window (ms).
    pub hesitation_window_ms: u64,
    
    // Motion thresholds  
    /// Minimum speed to consider "in motion" (m/s).
    pub motion_speed_threshold_mps: f32,
    /// Heading change to consider direction change (radians).
    pub direction_change_threshold_rad: f32,
    
    // Confidence thresholds
    /// Minimum confidence to emit disruption.
    pub min_confidence: f32,
    /// Minimum importance to emit disruption.
    pub min_importance: f32,
    
    // Searching behavior
    /// Number of direction changes in window to detect searching.
    pub searching_direction_changes: u32,
    /// Window for searching behavior detection (ms).
    pub searching_window_ms: u64,
}

impl Default for DisruptionConfig {
    fn default() -> Self {
        Self {
            min_motion_for_halt_ms: 5000,        // 5 seconds of motion
            extended_stillness_threshold_ms: 30000, // 30 seconds still
            hesitation_window_ms: 2000,          // 2 second pause
            motion_speed_threshold_mps: 0.3,     // 0.3 m/s
            direction_change_threshold_rad: 0.785, // ~45 degrees
            min_confidence: 0.5,
            min_importance: 0.3,
            searching_direction_changes: 3,
            searching_window_ms: 15000,          // 15 seconds
        }
    }
}

/// Rolling window for tracking recent history.
struct HistoryWindow {
    /// Recent positions (timestamp, x, y, heading).
    positions: Vec<(u64, f32, f32, f32)>,
    /// Recent steps (timestamp).
    steps: Vec<u64>,
    /// Recent micro-events (timestamp, type).
    micro_events: Vec<(u64, MicroEvent)>,
    /// Recent transport modes (timestamp, mode).
    transport_modes: Vec<(u64, TransportMode)>,
    /// Recent disruptions (timestamp).
    disruptions: Vec<u64>,
    /// Window duration to keep (ms).
    window_ms: u64,
}

impl HistoryWindow {
    fn new(window_ms: u64) -> Self {
        Self {
            positions: Vec::with_capacity(3000),
            steps: Vec::with_capacity(200),
            micro_events: Vec::with_capacity(50),
            transport_modes: Vec::with_capacity(20),
            disruptions: Vec::with_capacity(20),
            window_ms,
        }
    }

    fn add_position(&mut self, timestamp_ms: u64, x: f32, y: f32, heading: f32) {
        self.positions.push((timestamp_ms, x, y, heading));
        self.cleanup(timestamp_ms);
    }

    fn add_step(&mut self, timestamp_ms: u64) {
        self.steps.push(timestamp_ms);
    }

    fn add_micro_event(&mut self, timestamp_ms: u64, event: MicroEvent) {
        self.micro_events.push((timestamp_ms, event));
    }

    fn add_transport(&mut self, timestamp_ms: u64, mode: TransportMode) {
        self.transport_modes.push((timestamp_ms, mode));
    }

    fn add_disruption(&mut self, timestamp_ms: u64) {
        self.disruptions.push(timestamp_ms);
    }

    fn cleanup(&mut self, current_ms: u64) {
        let cutoff = current_ms.saturating_sub(self.window_ms);
        self.positions.retain(|(t, _, _, _)| *t > cutoff);
        self.steps.retain(|t| *t > cutoff);
        self.micro_events.retain(|(t, _)| *t > cutoff);
        self.transport_modes.retain(|(t, _)| *t > cutoff);
        self.disruptions.retain(|t| *t > cutoff);
    }

    fn steps_since(&self, timestamp_ms: u64, duration_ms: u64) -> u32 {
        let cutoff = timestamp_ms.saturating_sub(duration_ms);
        self.steps.iter().filter(|t| **t > cutoff).count() as u32
    }

    fn distance_since(&self, timestamp_ms: u64, duration_ms: u64) -> f32 {
        let cutoff = timestamp_ms.saturating_sub(duration_ms);
        let recent: Vec<_> = self.positions.iter()
            .filter(|(t, _, _, _)| *t > cutoff)
            .collect();
        
        if recent.len() < 2 {
            return 0.0;
        }
        
        let mut distance = 0.0f32;
        for i in 1..recent.len() {
            let dx = recent[i].1 - recent[i-1].1;
            let dy = recent[i].2 - recent[i-1].2;
            distance += (dx * dx + dy * dy).sqrt();
        }
        distance
    }

    fn heading_change_since(&self, timestamp_ms: u64, duration_ms: u64) -> f32 {
        let cutoff = timestamp_ms.saturating_sub(duration_ms);
        let recent: Vec<_> = self.positions.iter()
            .filter(|(t, _, _, _)| *t > cutoff)
            .collect();
        
        if recent.len() < 2 {
            return 0.0;
        }
        
        let mut total_change = 0.0f32;
        for i in 1..recent.len() {
            let mut delta = recent[i].3 - recent[i-1].3;
            // Normalize to [-π, π]
            while delta > std::f32::consts::PI {
                delta -= 2.0 * std::f32::consts::PI;
            }
            while delta < -std::f32::consts::PI {
                delta += 2.0 * std::f32::consts::PI;
            }
            total_change += delta.abs();
        }
        total_change
    }

    fn direction_changes_since(&self, timestamp_ms: u64, duration_ms: u64, threshold_rad: f32) -> u32 {
        let cutoff = timestamp_ms.saturating_sub(duration_ms);
        let recent: Vec<_> = self.positions.iter()
            .filter(|(t, _, _, _)| *t > cutoff)
            .collect();
        
        if recent.len() < 3 {
            return 0;
        }
        
        let mut changes = 0u32;
        let mut accumulated = 0.0f32;
        
        for i in 1..recent.len() {
            let mut delta = recent[i].3 - recent[i-1].3;
            while delta > std::f32::consts::PI {
                delta -= 2.0 * std::f32::consts::PI;
            }
            while delta < -std::f32::consts::PI {
                delta += 2.0 * std::f32::consts::PI;
            }
            accumulated += delta;
            
            if accumulated.abs() > threshold_rad {
                changes += 1;
                accumulated = 0.0;
            }
        }
        changes
    }

    fn recent_pickup(&self, timestamp_ms: u64, window_ms: u64) -> bool {
        let cutoff = timestamp_ms.saturating_sub(window_ms);
        self.micro_events.iter()
            .any(|(t, e)| *t > cutoff && *e == MicroEvent::PhonePickup)
    }

    fn recent_putdown(&self, timestamp_ms: u64, window_ms: u64) -> bool {
        let cutoff = timestamp_ms.saturating_sub(window_ms);
        self.micro_events.iter()
            .any(|(t, e)| *t > cutoff && *e == MicroEvent::PhonePutdown)
    }

    fn phone_interactions_since(&self, timestamp_ms: u64, duration_ms: u64) -> u32 {
        let cutoff = timestamp_ms.saturating_sub(duration_ms);
        self.micro_events.iter()
            .filter(|(t, e)| {
                *t > cutoff && matches!(e, 
                    MicroEvent::PhonePickup | 
                    MicroEvent::PhonePutdown |
                    MicroEvent::PocketIn |
                    MicroEvent::PocketOut)
            })
            .count() as u32
    }

    fn recent_transport_change(&self, timestamp_ms: u64, window_ms: u64) -> bool {
        let cutoff = timestamp_ms.saturating_sub(window_ms);
        let recent: Vec<_> = self.transport_modes.iter()
            .filter(|(t, _)| *t > cutoff)
            .collect();
        
        if recent.len() < 2 {
            return false;
        }
        
        recent.windows(2).any(|w| w[0].1 != w[1].1)
    }

    fn ms_since_last_disruption(&self, timestamp_ms: u64) -> u64 {
        self.disruptions.last()
            .map(|t| timestamp_ms.saturating_sub(*t))
            .unwrap_or(u64::MAX)
    }

    fn session_disruption_count(&self) -> u32 {
        self.disruptions.len() as u32
    }

    fn avg_speed_since(&self, timestamp_ms: u64, duration_ms: u64) -> f32 {
        let distance = self.distance_since(timestamp_ms, duration_ms);
        let duration_s = duration_ms as f32 / 1000.0;
        if duration_s > 0.0 {
            distance / duration_s
        } else {
            0.0
        }
    }

    fn motion_duration_before(&self, timestamp_ms: u64, speed_threshold: f32) -> f32 {
        // Walk backwards to find when motion started
        let mut motion_start = timestamp_ms;
        let mut prev_pos: Option<(u64, f32, f32)> = None;
        
        for (t, x, y, _) in self.positions.iter().rev() {
            if let Some((pt, px, py)) = prev_pos {
                let dt = (pt - *t) as f32 / 1000.0;
                if dt > 0.0 {
                    let dx = px - x;
                    let dy = py - y;
                    let speed = (dx * dx + dy * dy).sqrt() / dt;
                    
                    if speed >= speed_threshold {
                        motion_start = *t;
                    } else {
                        break;
                    }
                }
            }
            prev_pos = Some((*t, *x, *y));
        }
        
        (timestamp_ms - motion_start) as f32 / 1000.0
    }
}

/// Context Disruption Detector.
///
/// This is the key component that identifies moments when a user's
/// context may have been disrupted - feeding the CEBE-X engine.
pub struct ContextDisruptionDetector {
    config: DisruptionConfig,
    history: HistoryWindow,
    
    // State tracking
    current_position: TrajectoryPoint,
    current_transport: TransportMode,
    prev_transport: TransportMode,
    
    // Motion state
    is_moving: bool,
    motion_start_ms: u64,
    stillness_start_ms: u64,
    
    // Pending disruptions (for multi-sample confirmation)
    pending_halt: Option<u64>,
    pending_hesitation: Option<(u64, u64)>, // (start, end)
    
    // Statistics
    total_disruptions: u64,
}

impl ContextDisruptionDetector {
    /// Create a new context disruption detector.
    pub fn new(config: DisruptionConfig) -> Self {
        Self {
            config,
            history: HistoryWindow::new(120_000), // 2 minute history
            current_position: TrajectoryPoint::new(0),
            current_transport: TransportMode::Unknown,
            prev_transport: TransportMode::Unknown,
            is_moving: false,
            motion_start_ms: 0,
            stillness_start_ms: 0,
            pending_halt: None,
            pending_hesitation: None,
            total_disruptions: 0,
        }
    }

    /// Create with default configuration.
    pub fn default_detector() -> Self {
        Self::new(DisruptionConfig::default())
    }

    /// Update with trajectory and motion data.
    /// Call this every time the pipeline produces output.
    pub fn update(
        &mut self,
        timestamp_ms: u64,
        position: &TrajectoryPoint,
        transport_mode: TransportMode,
        step_event: Option<&StepEvent>,
        micro_events: &[MicroEventCandidate],
    ) -> Vec<ContextDisruption> {
        let mut disruptions = Vec::new();
        
        // Update history
        self.history.add_position(
            timestamp_ms, 
            position.x, 
            position.y, 
            position.heading_rad
        );
        
        if step_event.is_some() {
            self.history.add_step(timestamp_ms);
        }
        
        for event in micro_events {
            self.history.add_micro_event(timestamp_ms, event.event_type);
        }
        
        self.history.add_transport(timestamp_ms, transport_mode);
        
        // Detect speed for motion state
        let current_speed = position.velocity_mps;
        let was_moving = self.is_moving;
        self.is_moving = current_speed > self.config.motion_speed_threshold_mps;
        
        // Track motion/stillness transitions
        if self.is_moving && !was_moving {
            self.motion_start_ms = timestamp_ms;
        } else if !self.is_moving && was_moving {
            self.stillness_start_ms = timestamp_ms;
        }
        
        // === DISRUPTION DETECTION ===
        
        // 1. Phone placement - highest priority signal
        for event in micro_events {
            if event.event_type == MicroEvent::PhonePutdown && event.confidence > 0.5 {
                if let Some(d) = self.create_disruption(
                    timestamp_ms,
                    DisruptionType::PhonePlacement,
                    event.confidence,
                    position,
                    200,
                ) {
                    disruptions.push(d);
                }
            }
            
            // Phone pickup after placement = context resumption
            if event.event_type == MicroEvent::PhonePickup && event.confidence > 0.5 {
                if self.history.recent_putdown(timestamp_ms, 60_000) {
                    if let Some(d) = self.create_disruption(
                        timestamp_ms,
                        DisruptionType::ContextResumption,
                        event.confidence * 0.8,
                        position,
                        200,
                    ) {
                        disruptions.push(d);
                    }
                }
            }
        }
        
        // 2. Abrupt halt after sustained motion
        if !self.is_moving && was_moving {
            let motion_duration = self.stillness_start_ms.saturating_sub(self.motion_start_ms);
            if motion_duration >= self.config.min_motion_for_halt_ms {
                self.pending_halt = Some(timestamp_ms);
            }
        }
        
        // Confirm halt after brief stillness
        if let Some(halt_time) = self.pending_halt {
            let stillness_duration = timestamp_ms.saturating_sub(halt_time);
            if stillness_duration > 500 && !self.is_moving {
                // Confirmed halt
                if let Some(d) = self.create_disruption(
                    halt_time,
                    DisruptionType::AbruptHalt,
                    0.7,
                    position,
                    stillness_duration,
                ) {
                    disruptions.push(d);
                }
                self.pending_halt = None;
            } else if self.is_moving {
                // False alarm - started moving again
                self.pending_halt = None;
            }
        }
        
        // 3. Transport mode transition
        if transport_mode != self.prev_transport && self.prev_transport != TransportMode::Unknown {
            let importance = match (&self.prev_transport, &transport_mode) {
                (TransportMode::Vehicle, TransportMode::Walking) => 0.9, // Got off vehicle
                (TransportMode::Walking, TransportMode::Vehicle) => 0.7, // Got on vehicle
                (TransportMode::Stairs, _) | (_, TransportMode::Stairs) => 0.75,
                (TransportMode::Elevator, _) | (_, TransportMode::Elevator) => 0.8,
                _ => 0.6,
            };
            
            if let Some(d) = self.create_disruption(
                timestamp_ms,
                DisruptionType::TransportTransition,
                0.75,
                position,
                0,
            ) {
                let mut d = d;
                d.importance = importance;
                disruptions.push(d);
            }
        }
        
        // 4. Extended stillness
        if !self.is_moving {
            let stillness = timestamp_ms.saturating_sub(self.stillness_start_ms);
            if stillness >= self.config.extended_stillness_threshold_ms {
                // Only trigger once per stillness period
                if stillness < self.config.extended_stillness_threshold_ms + 1000 {
                    if let Some(d) = self.create_disruption(
                        self.stillness_start_ms,
                        DisruptionType::ExtendedStillness,
                        0.6,
                        position,
                        stillness,
                    ) {
                        disruptions.push(d);
                    }
                }
            }
        }
        
        // 5. Searching behavior (repeated direction changes)
        let dir_changes = self.history.direction_changes_since(
            timestamp_ms,
            self.config.searching_window_ms,
            self.config.direction_change_threshold_rad,
        );
        
        if dir_changes >= self.config.searching_direction_changes {
            // Check we haven't already flagged this
            let ms_since_last = self.history.ms_since_last_disruption(timestamp_ms);
            if ms_since_last > 10_000 {
                if let Some(d) = self.create_disruption(
                    timestamp_ms,
                    DisruptionType::SearchingBehavior,
                    0.65,
                    position,
                    self.config.searching_window_ms,
                ) {
                    disruptions.push(d);
                }
            }
        }
        
        // 6. Hesitation (brief pause during motion)
        if !self.is_moving && was_moving {
            let motion_duration = self.stillness_start_ms.saturating_sub(self.motion_start_ms);
            if motion_duration > 2000 && motion_duration < self.config.min_motion_for_halt_ms {
                self.pending_hesitation = Some((self.stillness_start_ms, timestamp_ms));
            }
        }
        
        if let Some((start, _)) = self.pending_hesitation {
            let pause_duration = timestamp_ms.saturating_sub(start);
            if self.is_moving && pause_duration < self.config.hesitation_window_ms {
                // Confirmed hesitation - paused briefly then resumed
                if let Some(d) = self.create_disruption(
                    start,
                    DisruptionType::Hesitation,
                    0.55,
                    position,
                    pause_duration,
                ) {
                    disruptions.push(d);
                }
                self.pending_hesitation = None;
            } else if pause_duration > self.config.hesitation_window_ms {
                // Too long - not a hesitation
                self.pending_hesitation = None;
            }
        }
        
        // 7. Environment transition (stairs/elevator)
        if transport_mode == TransportMode::Stairs || transport_mode == TransportMode::Elevator {
            if self.prev_transport != TransportMode::Stairs && 
               self.prev_transport != TransportMode::Elevator {
                if let Some(d) = self.create_disruption(
                    timestamp_ms,
                    DisruptionType::EnvironmentTransition,
                    0.7,
                    position,
                    0,
                ) {
                    disruptions.push(d);
                }
            }
        }
        
        // 8. Significant direction change (single large turn)
        let heading_change_10s = self.history.heading_change_since(timestamp_ms, 10_000);
        if heading_change_10s > std::f32::consts::PI * 0.75 { // > 135 degrees
            let ms_since_last = self.history.ms_since_last_disruption(timestamp_ms);
            if ms_since_last > 15_000 {
                if let Some(d) = self.create_disruption(
                    timestamp_ms,
                    DisruptionType::DirectionChange,
                    0.5,
                    position,
                    10_000,
                ) {
                    disruptions.push(d);
                }
            }
        }
        
        // Update state
        self.current_position = position.clone();
        self.prev_transport = transport_mode;
        self.current_transport = transport_mode;
        
        // Record disruptions in history
        for d in &disruptions {
            self.history.add_disruption(d.timestamp_ms);
            self.total_disruptions += 1;
        }
        
        disruptions
    }

    /// Get total disruptions detected.
    pub fn total_disruptions(&self) -> u64 {
        self.total_disruptions
    }

    /// Reset detector state.
    pub fn reset(&mut self) {
        self.history = HistoryWindow::new(120_000);
        self.is_moving = false;
        self.motion_start_ms = 0;
        self.stillness_start_ms = 0;
        self.pending_halt = None;
        self.pending_hesitation = None;
    }

    // === PRIVATE METHODS ===

    fn create_disruption(
        &self,
        timestamp_ms: u64,
        disruption_type: DisruptionType,
        confidence: f32,
        position: &TrajectoryPoint,
        duration_ms: u64,
    ) -> Option<ContextDisruption> {
        if confidence < self.config.min_confidence {
            return None;
        }
        
        let features = self.extract_features(timestamp_ms);
        let importance = self.compute_importance(disruption_type, &features);
        
        if importance < self.config.min_importance {
            return None;
        }
        
        Some(ContextDisruption {
            timestamp_ms,
            disruption_type,
            confidence,
            importance,
            position: position.clone(),
            duration_ms,
            features,
        })
    }

    fn extract_features(&self, timestamp_ms: u64) -> DisruptionFeatures {
        DisruptionFeatures {
            motion_duration_before_s: self.history.motion_duration_before(
                timestamp_ms, 
                self.config.motion_speed_threshold_mps
            ),
            avg_speed_before_mps: self.history.avg_speed_since(timestamp_ms, 30_000),
            steps_last_30s: self.history.steps_since(timestamp_ms, 30_000),
            distance_last_30s_m: self.history.distance_since(timestamp_ms, 30_000),
            heading_change_10s_rad: self.history.heading_change_since(timestamp_ms, 10_000),
            turns_last_30s: self.history.direction_changes_since(
                timestamp_ms, 
                30_000, 
                self.config.direction_change_threshold_rad
            ),
            stillness_before_s: if !self.is_moving { 0.0 } else {
                (timestamp_ms.saturating_sub(self.stillness_start_ms)) as f32 / 1000.0
            },
            stillness_after_s: 0.0, // Will be updated later if needed
            recent_pickup: self.history.recent_pickup(timestamp_ms, 30_000),
            recent_putdown: self.history.recent_putdown(timestamp_ms, 30_000),
            phone_interactions_60s: self.history.phone_interactions_since(timestamp_ms, 60_000),
            transport_mode: self.current_transport,
            recent_transport_change: self.history.recent_transport_change(timestamp_ms, 30_000),
            ms_since_last_disruption: self.history.ms_since_last_disruption(timestamp_ms),
            session_disruption_count: self.history.session_disruption_count(),
        }
    }

    fn compute_importance(&self, disruption_type: DisruptionType, features: &DisruptionFeatures) -> f32 {
        let mut importance = disruption_type.base_weight();
        
        // Boost importance based on context
        
        // More motion before = more important disruption
        if features.motion_duration_before_s > 30.0 {
            importance += 0.1;
        }
        
        // Recent phone interaction = higher importance
        if features.recent_putdown || features.recent_pickup {
            importance += 0.1;
        }
        
        // Transport change + halt = very important
        if features.recent_transport_change && 
           matches!(disruption_type, DisruptionType::AbruptHalt | DisruptionType::PhonePlacement) {
            importance += 0.15;
        }
        
        // First disruption in a while = more salient
        if features.ms_since_last_disruption > 60_000 {
            importance += 0.1;
        }
        
        // Too many recent disruptions = reduce importance (noise)
        if features.session_disruption_count > 10 {
            importance -= 0.1;
        }
        
        importance.clamp(0.0, 1.0)
    }
}

impl Default for ContextDisruptionDetector {
    fn default() -> Self {
        Self::default_detector()
    }
}


// ============================================================================
// SERIALIZATION FOR CEBE-X
// ============================================================================

impl ContextDisruption {
    /// Serialize to JSON for CEBE-X consumption.
    pub fn to_json(&self) -> String {
        format!(
            r#"{{"timestamp_ms":{},"type":"{}","confidence":{:.3},"importance":{:.3},"position":{{"x":{:.3},"y":{:.3},"z":{:.3},"heading_rad":{:.3}}},"duration_ms":{},"features":{}}}"#,
            self.timestamp_ms,
            self.disruption_type.description(),
            self.confidence,
            self.importance,
            self.position.x,
            self.position.y,
            self.position.z,
            self.position.heading_rad,
            self.duration_ms,
            self.features.to_json(),
        )
    }
}

impl DisruptionFeatures {
    /// Serialize to JSON for CEBE-X consumption.
    pub fn to_json(&self) -> String {
        format!(
            r#"{{"motion_duration_before_s":{:.2},"avg_speed_before_mps":{:.3},"steps_last_30s":{},"distance_last_30s_m":{:.2},"heading_change_10s_rad":{:.3},"turns_last_30s":{},"stillness_before_s":{:.2},"recent_pickup":{},"recent_putdown":{},"phone_interactions_60s":{},"transport_mode":"{}","recent_transport_change":{},"ms_since_last_disruption":{},"session_disruption_count":{}}}"#,
            self.motion_duration_before_s,
            self.avg_speed_before_mps,
            self.steps_last_30s,
            self.distance_last_30s_m,
            self.heading_change_10s_rad,
            self.turns_last_30s,
            self.stillness_before_s,
            self.recent_pickup,
            self.recent_putdown,
            self.phone_interactions_60s,
            format!("{:?}", self.transport_mode).to_lowercase(),
            self.recent_transport_change,
            self.ms_since_last_disruption,
            self.session_disruption_count,
        )
    }
}


// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detector_creation() {
        let detector = ContextDisruptionDetector::default_detector();
        assert_eq!(detector.total_disruptions(), 0);
    }

    #[test]
    fn test_disruption_type_weights() {
        assert!(DisruptionType::PhonePlacement.base_weight() > 0.8);
        assert!(DisruptionType::TransportTransition.base_weight() > 0.7);
        assert!(DisruptionType::Hesitation.base_weight() < 0.6);
    }

    #[test]
    fn test_phone_placement_detection() {
        let mut detector = ContextDisruptionDetector::default_detector();
        
        let position = TrajectoryPoint::new(1000);
        let micro_events = vec![
            MicroEventCandidate::new(1000, MicroEvent::PhonePutdown, 0.8, 200, 5.0)
        ];
        
        let disruptions = detector.update(
            1000,
            &position,
            TransportMode::Walking,
            None,
            &micro_events,
        );
        
        assert!(!disruptions.is_empty());
        assert_eq!(disruptions[0].disruption_type, DisruptionType::PhonePlacement);
    }

    #[test]
    fn test_json_serialization() {
        let features = DisruptionFeatures::default();
        let json = features.to_json();
        assert!(json.contains("motion_duration_before_s"));
        assert!(json.contains("transport_mode"));
    }

    #[test]
    fn test_history_window() {
        let mut history = HistoryWindow::new(10_000);
        
        for i in 0..100 {
            history.add_position(i * 100, i as f32 * 0.1, 0.0, 0.0);
        }
        
        // Should keep only last 10 seconds worth
        assert!(history.positions.len() <= 100);
        
        let distance = history.distance_since(9900, 5000);
        assert!(distance > 0.0);
    }

    #[test]
    fn test_importance_boosting() {
        let detector = ContextDisruptionDetector::default_detector();
        
        let mut features = DisruptionFeatures::default();
        features.motion_duration_before_s = 60.0; // Long motion
        features.recent_putdown = true;
        features.ms_since_last_disruption = 120_000; // 2 minutes
        
        let importance = detector.compute_importance(DisruptionType::AbruptHalt, &features);
        assert!(importance > DisruptionType::AbruptHalt.base_weight());
    }
}
