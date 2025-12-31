//! Export Module for Zone Mapper and CEBE-X
//!
//! This module provides structured JSON output that downstream services consume:
//! - **Zone Mapper**: Uses trajectory + position data to build spatial graphs
//! - **CEBE-X**: Uses disruption events + features for ML-based context reconstruction
//!
//! The export format is designed to be:
//! - Self-contained: All necessary context in each message
//! - Streamable: Can send incremental updates
//! - ML-friendly: Rich feature vectors for CEBE-X models

use crate::types::*;
use crate::context_disruption::ContextDisruption;

/// Complete session data for export.
#[derive(Debug, Clone)]
pub struct SessionExport {
    /// Session identifier (device + timestamp).
    pub session_id: String,
    /// Session start time (Unix ms).
    pub start_time_ms: u64,
    /// Session end time (Unix ms).
    pub end_time_ms: u64,
    /// Total duration (ms).
    pub duration_ms: u64,
    /// Device identifier.
    pub device_id: String,
    /// Trajectory points.
    pub trajectory: Vec<TrajectoryExport>,
    /// Movement episodes.
    pub episodes: Vec<EpisodeExport>,
    /// Context disruptions (sorted by importance).
    pub disruptions: Vec<DisruptionExport>,
    /// Session summary statistics.
    pub summary: SessionSummary,
}

/// Trajectory point for export.
#[derive(Debug, Clone)]
pub struct TrajectoryExport {
    pub timestamp_ms: u64,
    pub x_m: f32,
    pub y_m: f32,
    pub z_m: f32,
    pub heading_rad: f32,
    pub velocity_mps: f32,
    pub confidence: f32,
}

/// Movement episode for export.
#[derive(Debug, Clone)]
pub struct EpisodeExport {
    pub start_ms: u64,
    pub end_ms: u64,
    pub transport_mode: String,
    pub distance_m: f32,
    pub step_count: u32,
    pub avg_speed_mps: f32,
    pub heading_change_rad: f32,
}

/// Context disruption for export.
#[derive(Debug, Clone)]
pub struct DisruptionExport {
    pub timestamp_ms: u64,
    pub disruption_type: String,
    pub confidence: f32,
    pub importance: f32,
    pub position: PositionExport,
    pub duration_ms: u64,
    pub features: FeaturesExport,
    /// Rank in session (1 = most important).
    pub rank: u32,
}

/// Position for export.
#[derive(Debug, Clone)]
pub struct PositionExport {
    pub x_m: f32,
    pub y_m: f32,
    pub z_m: f32,
    pub heading_rad: f32,
}

/// Features for ML consumption.
#[derive(Debug, Clone)]
pub struct FeaturesExport {
    pub motion_duration_before_s: f32,
    pub avg_speed_before_mps: f32,
    pub steps_last_30s: u32,
    pub distance_last_30s_m: f32,
    pub heading_change_10s_rad: f32,
    pub turns_last_30s: u32,
    pub stillness_before_s: f32,
    pub recent_pickup: bool,
    pub recent_putdown: bool,
    pub phone_interactions_60s: u32,
    pub transport_mode: String,
    pub recent_transport_change: bool,
    pub ms_since_last_disruption: u64,
    pub session_disruption_count: u32,
}

/// Session summary statistics.
#[derive(Debug, Clone, Default)]
pub struct SessionSummary {
    pub total_distance_m: f32,
    pub total_steps: u32,
    pub total_disruptions: u32,
    pub top_disruption_importance: f32,
    pub transport_mode_breakdown: Vec<(String, f32)>, // (mode, percentage)
    pub avg_disruption_confidence: f32,
    pub movement_percentage: f32,
}

// ============================================================================
// JSON SERIALIZATION (zero-dependency)
// ============================================================================

impl SessionExport {
    /// Serialize to JSON string.
    pub fn to_json(&self) -> String {
        let mut json = String::with_capacity(8192);
        json.push('{');
        
        // Header
        json.push_str(&format!(r#""session_id":"{}","#, escape_json(&self.session_id)));
        json.push_str(&format!(r#""device_id":"{}","#, escape_json(&self.device_id)));
        json.push_str(&format!(r#""start_time_ms":{},"#, self.start_time_ms));
        json.push_str(&format!(r#""end_time_ms":{},"#, self.end_time_ms));
        json.push_str(&format!(r#""duration_ms":{},"#, self.duration_ms));
        
        // Summary
        json.push_str(&format!(r#""summary":{},"#, self.summary.to_json()));
        
        // Trajectory (can be large)
        json.push_str(r#""trajectory":["#);
        for (i, point) in self.trajectory.iter().enumerate() {
            if i > 0 { json.push(','); }
            json.push_str(&point.to_json());
        }
        json.push_str("],");
        
        // Episodes
        json.push_str(r#""episodes":["#);
        for (i, episode) in self.episodes.iter().enumerate() {
            if i > 0 { json.push(','); }
            json.push_str(&episode.to_json());
        }
        json.push_str("],");
        
        // Disruptions (sorted by rank)
        json.push_str(r#""disruptions":["#);
        for (i, disruption) in self.disruptions.iter().enumerate() {
            if i > 0 { json.push(','); }
            json.push_str(&disruption.to_json());
        }
        json.push(']');
        
        json.push('}');
        json
    }

    /// Serialize to compact JSON (minimal whitespace).
    pub fn to_compact_json(&self) -> String {
        self.to_json() // Already compact
    }

    /// Export only disruptions (for quick CEBE-X query).
    pub fn disruptions_only_json(&self) -> String {
        let mut json = String::with_capacity(2048);
        json.push('{');
        json.push_str(&format!(r#""session_id":"{}","#, escape_json(&self.session_id)));
        json.push_str(&format!(r#""device_id":"{}","#, escape_json(&self.device_id)));
        json.push_str(&format!(r#""disruption_count":{},"#, self.disruptions.len()));
        
        json.push_str(r#""disruptions":["#);
        for (i, disruption) in self.disruptions.iter().enumerate() {
            if i > 0 { json.push(','); }
            json.push_str(&disruption.to_json());
        }
        json.push(']');
        json.push('}');
        json
    }

    /// Export only trajectory (for Zone Mapper).
    pub fn trajectory_only_json(&self) -> String {
        let mut json = String::with_capacity(4096);
        json.push('{');
        json.push_str(&format!(r#""session_id":"{}","#, escape_json(&self.session_id)));
        json.push_str(&format!(r#""device_id":"{}","#, escape_json(&self.device_id)));
        json.push_str(&format!(r#""point_count":{},"#, self.trajectory.len()));
        
        json.push_str(r#""trajectory":["#);
        for (i, point) in self.trajectory.iter().enumerate() {
            if i > 0 { json.push(','); }
            json.push_str(&point.to_json());
        }
        json.push(']');
        json.push('}');
        json
    }
}

impl TrajectoryExport {
    fn to_json(&self) -> String {
        format!(
            r#"{{"t":{},"x":{:.3},"y":{:.3},"z":{:.3},"h":{:.4},"v":{:.3},"c":{:.2}}}"#,
            self.timestamp_ms,
            self.x_m,
            self.y_m,
            self.z_m,
            self.heading_rad,
            self.velocity_mps,
            self.confidence,
        )
    }
}

impl EpisodeExport {
    fn to_json(&self) -> String {
        format!(
            r#"{{"start_ms":{},"end_ms":{},"mode":"{}","distance_m":{:.2},"steps":{},"avg_speed_mps":{:.3},"heading_change_rad":{:.3}}}"#,
            self.start_ms,
            self.end_ms,
            self.transport_mode,
            self.distance_m,
            self.step_count,
            self.avg_speed_mps,
            self.heading_change_rad,
        )
    }
}

impl DisruptionExport {
    fn to_json(&self) -> String {
        format!(
            r#"{{"timestamp_ms":{},"type":"{}","confidence":{:.3},"importance":{:.3},"rank":{},"position":{},"duration_ms":{},"features":{}}}"#,
            self.timestamp_ms,
            self.disruption_type,
            self.confidence,
            self.importance,
            self.rank,
            self.position.to_json(),
            self.duration_ms,
            self.features.to_json(),
        )
    }
}

impl PositionExport {
    fn to_json(&self) -> String {
        format!(
            r#"{{"x":{:.3},"y":{:.3},"z":{:.3},"heading":{:.4}}}"#,
            self.x_m,
            self.y_m,
            self.z_m,
            self.heading_rad,
        )
    }
}

impl FeaturesExport {
    fn to_json(&self) -> String {
        format!(
            r#"{{"motion_before_s":{:.2},"speed_mps":{:.3},"steps_30s":{},"distance_30s_m":{:.2},"heading_change_10s":{:.3},"turns_30s":{},"still_before_s":{:.2},"pickup":{},"putdown":{},"phone_60s":{},"transport":"{}","transport_change":{},"ms_since_last":{},"disruption_count":{}}}"#,
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
            self.transport_mode,
            self.recent_transport_change,
            self.ms_since_last_disruption,
            self.session_disruption_count,
        )
    }
}

impl SessionSummary {
    fn to_json(&self) -> String {
        let mut modes = String::from("[");
        for (i, (mode, pct)) in self.transport_mode_breakdown.iter().enumerate() {
            if i > 0 { modes.push(','); }
            modes.push_str(&format!(r#"["{}",{:.2}]"#, mode, pct));
        }
        modes.push(']');
        
        format!(
            r#"{{"total_distance_m":{:.2},"total_steps":{},"total_disruptions":{},"top_importance":{:.3},"avg_confidence":{:.3},"movement_pct":{:.2},"transport_modes":{}}}"#,
            self.total_distance_m,
            self.total_steps,
            self.total_disruptions,
            self.top_disruption_importance,
            self.avg_disruption_confidence,
            self.movement_percentage,
            modes,
        )
    }
}

/// Escape JSON string.
fn escape_json(s: &str) -> String {
    let mut escaped = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"' => escaped.push_str("\\\""),
            '\\' => escaped.push_str("\\\\"),
            '\n' => escaped.push_str("\\n"),
            '\r' => escaped.push_str("\\r"),
            '\t' => escaped.push_str("\\t"),
            c if c.is_control() => escaped.push_str(&format!("\\u{:04x}", c as u32)),
            c => escaped.push(c),
        }
    }
    escaped
}

// ============================================================================
// SESSION BUILDER
// ============================================================================

/// Builds a session export from pipeline outputs.
pub struct SessionExportBuilder {
    session_id: String,
    device_id: String,
    start_time_ms: Option<u64>,
    trajectory: Vec<TrajectoryExport>,
    episodes: Vec<EpisodeExport>,
    disruptions: Vec<ContextDisruption>,
    
    // Tracking for summary
    total_steps: u32,
    last_timestamp_ms: u64,
    transport_time: [(TransportMode, u64); 6], // Time per mode
    moving_time_ms: u64,
}

impl SessionExportBuilder {
    /// Create a new session builder.
    pub fn new(device_id: &str) -> Self {
        Self {
            session_id: format!("{}_{}", device_id, timestamp_id()),
            device_id: device_id.to_string(),
            start_time_ms: None,
            trajectory: Vec::with_capacity(10_000),
            episodes: Vec::with_capacity(100),
            disruptions: Vec::with_capacity(50),
            total_steps: 0,
            last_timestamp_ms: 0,
            transport_time: [
                (TransportMode::Unknown, 0),
                (TransportMode::Stationary, 0),
                (TransportMode::Walking, 0),
                (TransportMode::Vehicle, 0),
                (TransportMode::Stairs, 0),
                (TransportMode::Elevator, 0),
            ],
            moving_time_ms: 0,
        }
    }

    /// Add a trajectory point.
    pub fn add_trajectory_point(&mut self, point: &TrajectoryPoint) {
        if self.start_time_ms.is_none() {
            self.start_time_ms = Some(point.timestamp_ms);
        }
        
        // Track time delta
        if self.last_timestamp_ms > 0 {
            let delta = point.timestamp_ms.saturating_sub(self.last_timestamp_ms);
            if point.velocity_mps > 0.1 {
                self.moving_time_ms += delta;
            }
        }
        self.last_timestamp_ms = point.timestamp_ms;
        
        self.trajectory.push(TrajectoryExport {
            timestamp_ms: point.timestamp_ms,
            x_m: point.x,
            y_m: point.y,
            z_m: point.z,
            heading_rad: point.heading_rad,
            velocity_mps: point.velocity_mps,
            confidence: point.confidence,
        });
    }

    /// Add a step event.
    pub fn add_step(&mut self) {
        self.total_steps += 1;
    }

    /// Add transport mode update.
    pub fn add_transport_mode(&mut self, timestamp_ms: u64, mode: TransportMode) {
        if self.last_timestamp_ms > 0 {
            let delta = timestamp_ms.saturating_sub(self.last_timestamp_ms);
            for (m, time) in &mut self.transport_time {
                if *m == mode {
                    *time += delta;
                    break;
                }
            }
        }
    }

    /// Add a movement episode.
    pub fn add_episode(&mut self, episode: &MovementEpisode) {
        let duration_s = (episode.end_ms - episode.start_ms) as f32 / 1000.0;
        let avg_speed = if duration_s > 0.0 { episode.distance_m / duration_s } else { 0.0 };
        
        self.episodes.push(EpisodeExport {
            start_ms: episode.start_ms,
            end_ms: episode.end_ms,
            transport_mode: format!("{:?}", episode.transport_mode).to_lowercase(),
            distance_m: episode.distance_m,
            step_count: episode.step_count,
            avg_speed_mps: avg_speed,
            heading_change_rad: episode.heading_change_rad,
        });
    }

    /// Add a context disruption.
    pub fn add_disruption(&mut self, disruption: ContextDisruption) {
        self.disruptions.push(disruption);
    }

    /// Build the final session export.
    pub fn build(mut self) -> SessionExport {
        // Sort disruptions by importance (descending)
        self.disruptions.sort_by(|a, b| {
            b.importance.partial_cmp(&a.importance).unwrap_or(std::cmp::Ordering::Equal)
        });
        
        // Convert to export format with ranks
        let disruption_exports: Vec<DisruptionExport> = self.disruptions
            .iter()
            .enumerate()
            .map(|(i, d)| disruption_to_export(d, (i + 1) as u32))
            .collect();
        
        let start_ms = self.start_time_ms.unwrap_or(0);
        let end_ms = self.last_timestamp_ms;
        let duration_ms = end_ms.saturating_sub(start_ms);
        
        // Compute summary
        let total_distance = self.trajectory.windows(2)
            .map(|w| {
                let dx = w[1].x_m - w[0].x_m;
                let dy = w[1].y_m - w[0].y_m;
                (dx * dx + dy * dy).sqrt()
            })
            .sum();
        
        let total_transport_time: u64 = self.transport_time.iter().map(|(_, t)| *t).sum();
        let transport_breakdown: Vec<(String, f32)> = if total_transport_time > 0 {
            self.transport_time.iter()
                .filter(|(_, t)| *t > 0)
                .map(|(m, t)| {
                    (format!("{:?}", m).to_lowercase(), *t as f32 / total_transport_time as f32 * 100.0)
                })
                .collect()
        } else {
            vec![]
        };
        
        let avg_confidence = if disruption_exports.is_empty() {
            0.0
        } else {
            disruption_exports.iter().map(|d| d.confidence).sum::<f32>() / disruption_exports.len() as f32
        };
        
        let top_importance = disruption_exports.first().map(|d| d.importance).unwrap_or(0.0);
        
        let movement_pct = if duration_ms > 0 {
            self.moving_time_ms as f32 / duration_ms as f32 * 100.0
        } else {
            0.0
        };
        
        let summary = SessionSummary {
            total_distance_m: total_distance,
            total_steps: self.total_steps,
            total_disruptions: disruption_exports.len() as u32,
            top_disruption_importance: top_importance,
            transport_mode_breakdown: transport_breakdown,
            avg_disruption_confidence: avg_confidence,
            movement_percentage: movement_pct,
        };
        
        SessionExport {
            session_id: self.session_id,
            device_id: self.device_id,
            start_time_ms: start_ms,
            end_time_ms: end_ms,
            duration_ms,
            trajectory: self.trajectory,
            episodes: self.episodes,
            disruptions: disruption_exports,
            summary,
        }
    }
}

fn disruption_to_export(d: &ContextDisruption, rank: u32) -> DisruptionExport {
    DisruptionExport {
        timestamp_ms: d.timestamp_ms,
        disruption_type: d.disruption_type.description().to_string(),
        confidence: d.confidence,
        importance: d.importance,
        rank,
        position: PositionExport {
            x_m: d.position.x,
            y_m: d.position.y,
            z_m: d.position.z,
            heading_rad: d.position.heading_rad,
        },
        duration_ms: d.duration_ms,
        features: FeaturesExport {
            motion_duration_before_s: d.features.motion_duration_before_s,
            avg_speed_before_mps: d.features.avg_speed_before_mps,
            steps_last_30s: d.features.steps_last_30s,
            distance_last_30s_m: d.features.distance_last_30s_m,
            heading_change_10s_rad: d.features.heading_change_10s_rad,
            turns_last_30s: d.features.turns_last_30s,
            stillness_before_s: d.features.stillness_before_s,
            recent_pickup: d.features.recent_pickup,
            recent_putdown: d.features.recent_putdown,
            phone_interactions_60s: d.features.phone_interactions_60s,
            transport_mode: format!("{:?}", d.features.transport_mode).to_lowercase(),
            recent_transport_change: d.features.recent_transport_change,
            ms_since_last_disruption: d.features.ms_since_last_disruption,
            session_disruption_count: d.features.session_disruption_count,
        },
    }
}

/// Generate a simple timestamp-based ID (no external deps).
fn timestamp_id() -> u64 {
    // In production, use proper timestamp. For now, use a counter.
    static COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}


// ============================================================================
// STREAMING EXPORT (for real-time updates to Zone Mapper)
// ============================================================================

/// A single streaming update message.
#[derive(Debug)]
pub struct StreamingUpdate {
    pub sequence: u64,
    pub timestamp_ms: u64,
    pub update_type: StreamingUpdateType,
}

#[derive(Debug)]
pub enum StreamingUpdateType {
    /// New trajectory point.
    TrajectoryPoint(TrajectoryExport),
    /// Step detected.
    Step { timestamp_ms: u64 },
    /// Transport mode changed.
    TransportChange { mode: String, confidence: f32 },
    /// Context disruption detected.
    Disruption(DisruptionExport),
    /// Episode ended.
    EpisodeComplete(EpisodeExport),
}

impl StreamingUpdate {
    pub fn to_json(&self) -> String {
        let inner = match &self.update_type {
            StreamingUpdateType::TrajectoryPoint(p) => {
                format!(r#""type":"trajectory","data":{}"#, p.to_json())
            }
            StreamingUpdateType::Step { timestamp_ms } => {
                format!(r#""type":"step","data":{{"t":{}}}"#, timestamp_ms)
            }
            StreamingUpdateType::TransportChange { mode, confidence } => {
                format!(r#""type":"transport","data":{{"mode":"{}","confidence":{:.2}}}"#, mode, confidence)
            }
            StreamingUpdateType::Disruption(d) => {
                format!(r#""type":"disruption","data":{}"#, d.to_json())
            }
            StreamingUpdateType::EpisodeComplete(e) => {
                format!(r#""type":"episode","data":{}"#, e.to_json())
            }
        };
        
        format!(r#"{{"seq":{},"ts":{},{}}}"#, self.sequence, self.timestamp_ms, inner)
    }
}

/// Streaming export manager.
pub struct StreamingExporter {
    sequence: u64,
    device_id: String,
}

impl StreamingExporter {
    pub fn new(device_id: &str) -> Self {
        Self {
            sequence: 0,
            device_id: device_id.to_string(),
        }
    }

    pub fn trajectory_update(&mut self, point: &TrajectoryPoint) -> StreamingUpdate {
        self.sequence += 1;
        StreamingUpdate {
            sequence: self.sequence,
            timestamp_ms: point.timestamp_ms,
            update_type: StreamingUpdateType::TrajectoryPoint(TrajectoryExport {
                timestamp_ms: point.timestamp_ms,
                x_m: point.x,
                y_m: point.y,
                z_m: point.z,
                heading_rad: point.heading_rad,
                velocity_mps: point.velocity_mps,
                confidence: point.confidence,
            }),
        }
    }

    pub fn step_update(&mut self, timestamp_ms: u64) -> StreamingUpdate {
        self.sequence += 1;
        StreamingUpdate {
            sequence: self.sequence,
            timestamp_ms,
            update_type: StreamingUpdateType::Step { timestamp_ms },
        }
    }

    pub fn disruption_update(&mut self, d: &ContextDisruption) -> StreamingUpdate {
        self.sequence += 1;
        StreamingUpdate {
            sequence: self.sequence,
            timestamp_ms: d.timestamp_ms,
            update_type: StreamingUpdateType::Disruption(disruption_to_export(d, 0)),
        }
    }

    pub fn device_id(&self) -> &str {
        &self.device_id
    }

    pub fn sequence(&self) -> u64 {
        self.sequence
    }
}


// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_session_builder() {
        let mut builder = SessionExportBuilder::new("test_device");
        
        // Add some trajectory points
        for i in 0..10 {
            let mut point = TrajectoryPoint::new(i * 100);
            point.x = i as f32 * 0.5;
            point.y = i as f32 * 0.3;
            builder.add_trajectory_point(&point);
        }
        
        let session = builder.build();
        assert_eq!(session.trajectory.len(), 10);
        assert!(!session.session_id.is_empty());
    }

    #[test]
    fn test_json_serialization() {
        let mut builder = SessionExportBuilder::new("device_001");
        
        let mut point = TrajectoryPoint::new(1000);
        point.x = 1.0;
        point.y = 2.0;
        builder.add_trajectory_point(&point);
        
        let session = builder.build();
        let json = session.to_json();
        
        assert!(json.contains("session_id"));
        assert!(json.contains("device_001"));
        assert!(json.contains("trajectory"));
    }

    #[test]
    fn test_json_escaping() {
        let escaped = escape_json("test\"with\\quotes");
        assert_eq!(escaped, r#"test\"with\\quotes"#);
    }

    #[test]
    fn test_streaming_exporter() {
        let mut exporter = StreamingExporter::new("device_001");
        
        let point = TrajectoryPoint::new(1000);
        let update = exporter.trajectory_update(&point);
        
        let json = update.to_json();
        assert!(json.contains("seq"));
        assert!(json.contains("trajectory"));
    }

    #[test]
    fn test_disruptions_only_export() {
        let builder = SessionExportBuilder::new("device_001");
        let session = builder.build();
        let json = session.disruptions_only_json();
        
        assert!(json.contains("disruptions"));
        assert!(json.contains("device_001"));
    }

    #[test]
    fn test_summary_computation() {
        let mut builder = SessionExportBuilder::new("test");
        
        for i in 0..100 {
            let mut point = TrajectoryPoint::new(i * 100);
            point.x = i as f32 * 0.1;
            point.velocity_mps = 1.0;
            builder.add_trajectory_point(&point);
            builder.add_step();
        }
        
        let session = builder.build();
        assert_eq!(session.summary.total_steps, 100);
        assert!(session.summary.total_distance_m > 0.0);
    }
}
