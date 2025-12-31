//! Binary Wire Format for Zero-Copy Data Exchange
//!
//! Production-grade serialization for the Trace IMU Engine.
//! 
//! Design principles:
//! - **Zero-copy reads**: Consumer can read directly from buffer
//! - **Fixed-size headers**: O(1) to find any field
//! - **No allocations**: Write into pre-allocated buffers
//! - **Little-endian**: Native for ARM and x86
//! - **Aligned**: 4-byte alignment for direct memory mapping
//!
//! Wire format is designed for:
//! - Zone Mapper: Streaming trajectory updates over Unix socket / shared memory
//! - CEBE-X: Batch disruption export over gRPC / HTTP/2
//! - React Native: Direct FFI buffer access
//!
//! # Message Types
//!
//! | Type ID | Name | Size | Description |
//! |---------|------|------|-------------|
//! | 0x01 | TrajectoryUpdate | 32 bytes | Single position update |
//! | 0x02 | StepEvent | 20 bytes | Step detected |
//! | 0x03 | DisruptionEvent | 64 bytes | Context disruption |
//! | 0x04 | TransportChange | 12 bytes | Mode transition |
//! | 0x05 | SessionSummary | 48 bytes | End-of-session stats |
//!
//! # Header Format (8 bytes)
//!
//! ```text
//! ┌────────┬────────┬────────────────┬────────────────┐
//! │ Magic  │ Type   │ Sequence       │ Payload Length │
//! │ 2 bytes│ 1 byte │ 1 byte (unused)│ 4 bytes        │
//! └────────┴────────┴────────────────┴────────────────┘
//! ```

use crate::types::{TrajectoryPoint, StepEvent, TransportMode, StancePhase};
use crate::context_disruption::{ContextDisruption, DisruptionType};

// ============================================================================
// CONSTANTS
// ============================================================================

/// Magic bytes for message validation: "TR" (0x5452)
pub const WIRE_MAGIC: u16 = 0x5452;

/// Wire format version
pub const WIRE_VERSION: u8 = 1;

/// Message type identifiers
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    TrajectoryUpdate = 0x01,
    StepEvent = 0x02,
    DisruptionEvent = 0x03,
    TransportChange = 0x04,
    SessionSummary = 0x05,
    Heartbeat = 0xFF,
}

impl MessageType {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(MessageType::TrajectoryUpdate),
            0x02 => Some(MessageType::StepEvent),
            0x03 => Some(MessageType::DisruptionEvent),
            0x04 => Some(MessageType::TransportChange),
            0x05 => Some(MessageType::SessionSummary),
            0xFF => Some(MessageType::Heartbeat),
            _ => None,
        }
    }
}

// ============================================================================
// WIRE HEADER (8 bytes, fixed)
// ============================================================================

/// Message header - fixed 8 bytes, 4-byte aligned
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct WireHeader {
    /// Magic bytes for validation (0x5452 = "TR")
    pub magic: u16,
    /// Message type
    pub msg_type: u8,
    /// Reserved for future use / alignment
    pub _reserved: u8,
    /// Payload length in bytes (excludes header)
    pub payload_len: u32,
}

impl WireHeader {
    pub const SIZE: usize = 8;

    #[inline]
    pub fn new(msg_type: MessageType, payload_len: u32) -> Self {
        Self {
            magic: WIRE_MAGIC,
            msg_type: msg_type as u8,
            _reserved: 0,
            payload_len,
        }
    }

    /// Write header to buffer. Returns bytes written (always 8).
    #[inline]
    pub fn write_to(&self, buf: &mut [u8]) -> usize {
        if buf.len() < Self::SIZE {
            return 0;
        }
        buf[0..2].copy_from_slice(&self.magic.to_le_bytes());
        buf[2] = self.msg_type;
        buf[3] = self._reserved;
        buf[4..8].copy_from_slice(&self.payload_len.to_le_bytes());
        Self::SIZE
    }

    /// Read header from buffer.
    #[inline]
    pub fn read_from(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let magic = u16::from_le_bytes([buf[0], buf[1]]);
        if magic != WIRE_MAGIC {
            return None;
        }
        Some(Self {
            magic,
            msg_type: buf[2],
            _reserved: buf[3],
            payload_len: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        })
    }

    #[inline]
    pub fn message_type(&self) -> Option<MessageType> {
        MessageType::from_u8(self.msg_type)
    }
}

// ============================================================================
// TRAJECTORY UPDATE (32 bytes payload)
// ============================================================================

/// Trajectory point wire format - 32 bytes, zero-copy readable
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WireTrajectory {
    /// Timestamp in milliseconds
    pub timestamp_ms: u64,      // 8 bytes
    /// X position in meters (f32 as u32 bits)
    pub x_bits: u32,            // 4 bytes
    /// Y position in meters
    pub y_bits: u32,            // 4 bytes
    /// Z position in meters (altitude)
    pub z_bits: u32,            // 4 bytes
    /// Heading in radians
    pub heading_bits: u32,      // 4 bytes
    /// Velocity in m/s
    pub velocity_bits: u32,     // 4 bytes
    /// Confidence [0-255] mapped to [0.0-1.0]
    pub confidence: u8,         // 1 byte
    /// Stance phase (0=Unknown, 1=Stance, 2=Swing)
    pub stance: u8,             // 1 byte
    /// Padding for alignment
    pub _pad: u16,              // 2 bytes
}                               // Total: 32 bytes

impl WireTrajectory {
    pub const SIZE: usize = 32;

    #[inline]
    pub fn from_trajectory(t: &TrajectoryPoint) -> Self {
        Self {
            timestamp_ms: t.timestamp_ms,
            x_bits: t.x.to_bits(),
            y_bits: t.y.to_bits(),
            z_bits: t.z.to_bits(),
            heading_bits: t.heading_rad.to_bits(),
            velocity_bits: t.velocity_mps.to_bits(),
            confidence: (t.confidence * 255.0) as u8,
            stance: match t.stance_phase {
                StancePhase::Unknown => 0,
                StancePhase::Stance => 1,
                StancePhase::Swing => 2,
            },
            _pad: 0,
        }
    }

    /// Write to buffer. Returns bytes written.
    #[inline]
    pub fn write_to(&self, buf: &mut [u8]) -> usize {
        if buf.len() < Self::SIZE {
            return 0;
        }
        buf[0..8].copy_from_slice(&self.timestamp_ms.to_le_bytes());
        buf[8..12].copy_from_slice(&self.x_bits.to_le_bytes());
        buf[12..16].copy_from_slice(&self.y_bits.to_le_bytes());
        buf[16..20].copy_from_slice(&self.z_bits.to_le_bytes());
        buf[20..24].copy_from_slice(&self.heading_bits.to_le_bytes());
        buf[24..28].copy_from_slice(&self.velocity_bits.to_le_bytes());
        buf[28] = self.confidence;
        buf[29] = self.stance;
        buf[30..32].copy_from_slice(&self._pad.to_le_bytes());
        Self::SIZE
    }

    /// Read from buffer (zero-copy where possible).
    #[inline]
    pub fn read_from(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            timestamp_ms: u64::from_le_bytes([
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]
            ]),
            x_bits: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            y_bits: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
            z_bits: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            heading_bits: u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
            velocity_bits: u32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]),
            confidence: buf[28],
            stance: buf[29],
            _pad: u16::from_le_bytes([buf[30], buf[31]]),
        })
    }

    /// Get X position as f32
    #[inline]
    pub fn x(&self) -> f32 { f32::from_bits(self.x_bits) }
    /// Get Y position as f32
    #[inline]
    pub fn y(&self) -> f32 { f32::from_bits(self.y_bits) }
    /// Get Z position as f32
    #[inline]
    pub fn z(&self) -> f32 { f32::from_bits(self.z_bits) }
    /// Get heading as f32
    #[inline]
    pub fn heading(&self) -> f32 { f32::from_bits(self.heading_bits) }
    /// Get velocity as f32
    #[inline]
    pub fn velocity(&self) -> f32 { f32::from_bits(self.velocity_bits) }
    /// Get confidence as f32 [0.0-1.0]
    #[inline]
    pub fn confidence_f32(&self) -> f32 { self.confidence as f32 / 255.0 }
}

// ============================================================================
// STEP EVENT (20 bytes payload)
// ============================================================================

/// Step event wire format - 20 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WireStep {
    /// Timestamp in milliseconds
    pub timestamp_ms: u64,      // 8 bytes
    /// Stride length in centimeters (0-655cm range)
    pub stride_cm: u16,         // 2 bytes
    /// Cadence in steps per minute
    pub cadence_spm: u16,       // 2 bytes
    /// Step confidence [0-255]
    pub confidence: u8,         // 1 byte
    /// Cumulative step count (lower 24 bits)
    pub step_count: [u8; 3],    // 3 bytes
    /// Heading at step (radians, scaled to u16)
    pub heading_u16: u16,       // 2 bytes
    /// Padding
    pub _pad: u16,              // 2 bytes
}                               // Total: 20 bytes

impl WireStep {
    pub const SIZE: usize = 20;

    #[inline]
    pub fn from_step(step: &StepEvent, cumulative_count: u32, current_heading_rad: f32) -> Self {
        // Convert step_frequency_hz to cadence (steps per minute)
        let cadence = step.step_frequency_hz * 60.0;
        Self {
            timestamp_ms: step.timestamp_ms,
            stride_cm: (step.stride_length_m * 100.0).clamp(0.0, 65535.0) as u16,
            cadence_spm: cadence.clamp(0.0, 65535.0) as u16,
            confidence: (step.confidence * 255.0) as u8,
            step_count: [
                (cumulative_count & 0xFF) as u8,
                ((cumulative_count >> 8) & 0xFF) as u8,
                ((cumulative_count >> 16) & 0xFF) as u8,
            ],
            heading_u16: ((current_heading_rad + std::f32::consts::PI) / (2.0 * std::f32::consts::PI) * 65535.0) as u16,
            _pad: 0,
        }
    }

    #[inline]
    pub fn write_to(&self, buf: &mut [u8]) -> usize {
        if buf.len() < Self::SIZE {
            return 0;
        }
        buf[0..8].copy_from_slice(&self.timestamp_ms.to_le_bytes());
        buf[8..10].copy_from_slice(&self.stride_cm.to_le_bytes());
        buf[10..12].copy_from_slice(&self.cadence_spm.to_le_bytes());
        buf[12] = self.confidence;
        buf[13..16].copy_from_slice(&self.step_count);
        buf[16..18].copy_from_slice(&self.heading_u16.to_le_bytes());
        buf[18..20].copy_from_slice(&self._pad.to_le_bytes());
        Self::SIZE
    }

    #[inline]
    pub fn read_from(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            timestamp_ms: u64::from_le_bytes([
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]
            ]),
            stride_cm: u16::from_le_bytes([buf[8], buf[9]]),
            cadence_spm: u16::from_le_bytes([buf[10], buf[11]]),
            confidence: buf[12],
            step_count: [buf[13], buf[14], buf[15]],
            heading_u16: u16::from_le_bytes([buf[16], buf[17]]),
            _pad: u16::from_le_bytes([buf[18], buf[19]]),
        })
    }

    /// Get stride in meters
    #[inline]
    pub fn stride_m(&self) -> f32 { self.stride_cm as f32 / 100.0 }
    
    /// Get heading in radians
    #[inline]
    pub fn heading_rad(&self) -> f32 {
        (self.heading_u16 as f32 / 65535.0) * 2.0 * std::f32::consts::PI - std::f32::consts::PI
    }
    
    /// Get cumulative step count
    #[inline]
    pub fn count(&self) -> u32 {
        self.step_count[0] as u32 
            | ((self.step_count[1] as u32) << 8)
            | ((self.step_count[2] as u32) << 16)
    }
}

// ============================================================================
// DISRUPTION EVENT (64 bytes payload)
// ============================================================================

/// Context disruption wire format - 64 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WireDisruption {
    /// Timestamp in milliseconds
    pub timestamp_ms: u64,          // 8 bytes
    /// Position X in meters
    pub x_bits: u32,                // 4 bytes
    /// Position Y in meters
    pub y_bits: u32,                // 4 bytes
    /// Position Z in meters
    pub z_bits: u32,                // 4 bytes
    /// Heading in radians
    pub heading_bits: u32,          // 4 bytes
    /// Disruption type (see DisruptionType enum)
    pub disruption_type: u8,        // 1 byte
    /// Confidence [0-255]
    pub confidence: u8,             // 1 byte
    /// Importance [0-255]
    pub importance: u8,             // 1 byte
    /// Transport mode at disruption
    pub transport_mode: u8,         // 1 byte
    /// Duration of disruption event (ms, capped at 65535)
    pub duration_ms: u16,           // 2 bytes
    /// Motion duration before (seconds, capped at 65535)
    pub motion_before_s: u16,       // 2 bytes
    /// Steps in last 30 seconds
    pub steps_30s: u16,             // 2 bytes
    /// Distance in last 30 seconds (decimeters)
    pub distance_30s_dm: u16,       // 2 bytes
    /// Heading change in last 10s (scaled)
    pub heading_change_u16: u16,    // 2 bytes
    /// Turn count in last 30s
    pub turns_30s: u8,              // 1 byte
    /// Flags: bit 0 = recent_pickup, bit 1 = recent_putdown, bit 2 = recent_transport_change
    pub flags: u8,                  // 1 byte
    /// Phone interactions in last 60s
    pub phone_interactions_60s: u8, // 1 byte
    /// Session disruption count
    pub session_disruption_count: u8, // 1 byte
    /// Milliseconds since last disruption (capped at 65535)
    pub ms_since_last: u16,         // 2 bytes
    /// Rank (1 = most important)
    pub rank: u16,                  // 2 bytes
    /// Reserved for future features
    pub _reserved: [u8; 12],        // 12 bytes
}                                   // Total: 64 bytes

impl WireDisruption {
    pub const SIZE: usize = 64;

    #[inline]
    pub fn from_disruption(d: &ContextDisruption, rank: u16) -> Self {
        let flags = 
            (if d.features.recent_pickup { 1 } else { 0 })
            | (if d.features.recent_putdown { 2 } else { 0 })
            | (if d.features.recent_transport_change { 4 } else { 0 });

        Self {
            timestamp_ms: d.timestamp_ms,
            x_bits: d.position.x.to_bits(),
            y_bits: d.position.y.to_bits(),
            z_bits: d.position.z.to_bits(),
            heading_bits: d.position.heading_rad.to_bits(),
            disruption_type: match d.disruption_type {
                DisruptionType::PhonePlacement => 0,
                DisruptionType::AbruptHalt => 1,
                DisruptionType::DirectionChange => 2,
                DisruptionType::TransportTransition => 3,
                DisruptionType::Hesitation => 4,
                DisruptionType::ContextResumption => 5,
                DisruptionType::ExtendedStillness => 6,
                DisruptionType::SearchingBehavior => 7,
                DisruptionType::EnvironmentTransition => 8,
            },
            confidence: (d.confidence * 255.0) as u8,
            importance: (d.importance * 255.0) as u8,
            transport_mode: match d.features.transport_mode {
                TransportMode::Unknown => 0,
                TransportMode::Stationary => 1,
                TransportMode::Walking => 2,
                TransportMode::Running => 3,
                TransportMode::Vehicle => 4,
                TransportMode::Cycling => 5,
                TransportMode::Stairs => 6,
                TransportMode::Elevator => 7,
            },
            duration_ms: d.duration_ms.min(65535) as u16,
            motion_before_s: d.features.motion_duration_before_s.clamp(0.0, 65535.0) as u16,
            steps_30s: d.features.steps_last_30s.min(65535) as u16,
            distance_30s_dm: (d.features.distance_last_30s_m * 10.0).clamp(0.0, 65535.0) as u16,
            heading_change_u16: ((d.features.heading_change_10s_rad + std::f32::consts::PI) 
                / (2.0 * std::f32::consts::PI) * 65535.0) as u16,
            turns_30s: d.features.turns_last_30s.min(255) as u8,
            flags,
            phone_interactions_60s: d.features.phone_interactions_60s.min(255) as u8,
            session_disruption_count: d.features.session_disruption_count.min(255) as u8,
            ms_since_last: d.features.ms_since_last_disruption.min(65535) as u16,
            rank,
            _reserved: [0; 12],
        }
    }

    #[inline]
    pub fn write_to(&self, buf: &mut [u8]) -> usize {
        if buf.len() < Self::SIZE {
            return 0;
        }
        buf[0..8].copy_from_slice(&self.timestamp_ms.to_le_bytes());
        buf[8..12].copy_from_slice(&self.x_bits.to_le_bytes());
        buf[12..16].copy_from_slice(&self.y_bits.to_le_bytes());
        buf[16..20].copy_from_slice(&self.z_bits.to_le_bytes());
        buf[20..24].copy_from_slice(&self.heading_bits.to_le_bytes());
        buf[24] = self.disruption_type;
        buf[25] = self.confidence;
        buf[26] = self.importance;
        buf[27] = self.transport_mode;
        buf[28..30].copy_from_slice(&self.duration_ms.to_le_bytes());
        buf[30..32].copy_from_slice(&self.motion_before_s.to_le_bytes());
        buf[32..34].copy_from_slice(&self.steps_30s.to_le_bytes());
        buf[34..36].copy_from_slice(&self.distance_30s_dm.to_le_bytes());
        buf[36..38].copy_from_slice(&self.heading_change_u16.to_le_bytes());
        buf[38] = self.turns_30s;
        buf[39] = self.flags;
        buf[40] = self.phone_interactions_60s;
        buf[41] = self.session_disruption_count;
        buf[42..44].copy_from_slice(&self.ms_since_last.to_le_bytes());
        buf[44..46].copy_from_slice(&self.rank.to_le_bytes());
        buf[46..58].copy_from_slice(&self._reserved);
        // Ensure we write exactly 64 bytes
        buf[58..64].copy_from_slice(&[0u8; 6]);
        Self::SIZE
    }

    #[inline]
    pub fn read_from(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let mut reserved = [0u8; 12];
        reserved.copy_from_slice(&buf[46..58]);
        
        Some(Self {
            timestamp_ms: u64::from_le_bytes([
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]
            ]),
            x_bits: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            y_bits: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
            z_bits: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            heading_bits: u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
            disruption_type: buf[24],
            confidence: buf[25],
            importance: buf[26],
            transport_mode: buf[27],
            duration_ms: u16::from_le_bytes([buf[28], buf[29]]),
            motion_before_s: u16::from_le_bytes([buf[30], buf[31]]),
            steps_30s: u16::from_le_bytes([buf[32], buf[33]]),
            distance_30s_dm: u16::from_le_bytes([buf[34], buf[35]]),
            heading_change_u16: u16::from_le_bytes([buf[36], buf[37]]),
            turns_30s: buf[38],
            flags: buf[39],
            phone_interactions_60s: buf[40],
            session_disruption_count: buf[41],
            ms_since_last: u16::from_le_bytes([buf[42], buf[43]]),
            rank: u16::from_le_bytes([buf[44], buf[45]]),
            _reserved: reserved,
        })
    }

    /// Get X position
    #[inline] pub fn x(&self) -> f32 { f32::from_bits(self.x_bits) }
    /// Get Y position
    #[inline] pub fn y(&self) -> f32 { f32::from_bits(self.y_bits) }
    /// Get Z position
    #[inline] pub fn z(&self) -> f32 { f32::from_bits(self.z_bits) }
    /// Get heading
    #[inline] pub fn heading(&self) -> f32 { f32::from_bits(self.heading_bits) }
    /// Get confidence [0.0-1.0]
    #[inline] pub fn confidence_f32(&self) -> f32 { self.confidence as f32 / 255.0 }
    /// Get importance [0.0-1.0]
    #[inline] pub fn importance_f32(&self) -> f32 { self.importance as f32 / 255.0 }
    /// Get distance in meters
    #[inline] pub fn distance_30s_m(&self) -> f32 { self.distance_30s_dm as f32 / 10.0 }
    /// Check if recent pickup
    #[inline] pub fn recent_pickup(&self) -> bool { self.flags & 1 != 0 }
    /// Check if recent putdown
    #[inline] pub fn recent_putdown(&self) -> bool { self.flags & 2 != 0 }
    /// Check if recent transport change
    #[inline] pub fn recent_transport_change(&self) -> bool { self.flags & 4 != 0 }
}

// ============================================================================
// TRANSPORT CHANGE (12 bytes payload)
// ============================================================================

/// Transport mode change wire format - 12 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WireTransport {
    /// Timestamp in milliseconds
    pub timestamp_ms: u64,  // 8 bytes
    /// Previous mode
    pub from_mode: u8,      // 1 byte
    /// New mode
    pub to_mode: u8,        // 1 byte
    /// Confidence [0-255]
    pub confidence: u8,     // 1 byte
    /// Padding
    pub _pad: u8,           // 1 byte
}                           // Total: 12 bytes

impl WireTransport {
    pub const SIZE: usize = 12;

    #[inline]
    pub fn new(timestamp_ms: u64, from: TransportMode, to: TransportMode, confidence: f32) -> Self {
        Self {
            timestamp_ms,
            from_mode: transport_to_u8(from),
            to_mode: transport_to_u8(to),
            confidence: (confidence * 255.0) as u8,
            _pad: 0,
        }
    }

    #[inline]
    pub fn write_to(&self, buf: &mut [u8]) -> usize {
        if buf.len() < Self::SIZE {
            return 0;
        }
        buf[0..8].copy_from_slice(&self.timestamp_ms.to_le_bytes());
        buf[8] = self.from_mode;
        buf[9] = self.to_mode;
        buf[10] = self.confidence;
        buf[11] = self._pad;
        Self::SIZE
    }

    #[inline]
    pub fn read_from(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            timestamp_ms: u64::from_le_bytes([
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]
            ]),
            from_mode: buf[8],
            to_mode: buf[9],
            confidence: buf[10],
            _pad: buf[11],
        })
    }
}

// ============================================================================
// SESSION SUMMARY (48 bytes payload)
// ============================================================================

/// End-of-session summary wire format - 48 bytes
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WireSessionSummary {
    /// Session start timestamp (ms)
    pub start_ms: u64,              // 8 bytes
    /// Session end timestamp (ms)
    pub end_ms: u64,                // 8 bytes
    /// Total distance in centimeters
    pub total_distance_cm: u32,     // 4 bytes
    /// Total step count
    pub total_steps: u32,           // 4 bytes
    /// Total disruption count
    pub total_disruptions: u16,     // 2 bytes
    /// Top disruption importance [0-255]
    pub top_importance: u8,         // 1 byte
    /// Primary transport mode
    pub primary_transport: u8,      // 1 byte
    /// Percentage of time moving [0-100]
    pub movement_pct: u8,           // 1 byte
    /// Average disruption confidence [0-255]
    pub avg_disruption_conf: u8,    // 1 byte
    /// Reserved
    pub _reserved: [u8; 18],        // 18 bytes
}                                   // Total: 48 bytes

impl WireSessionSummary {
    pub const SIZE: usize = 48;

    #[inline]
    pub fn write_to(&self, buf: &mut [u8]) -> usize {
        if buf.len() < Self::SIZE {
            return 0;
        }
        buf[0..8].copy_from_slice(&self.start_ms.to_le_bytes());
        buf[8..16].copy_from_slice(&self.end_ms.to_le_bytes());
        buf[16..20].copy_from_slice(&self.total_distance_cm.to_le_bytes());
        buf[20..24].copy_from_slice(&self.total_steps.to_le_bytes());
        buf[24..26].copy_from_slice(&self.total_disruptions.to_le_bytes());
        buf[26] = self.top_importance;
        buf[27] = self.primary_transport;
        buf[28] = self.movement_pct;
        buf[29] = self.avg_disruption_conf;
        buf[30..48].copy_from_slice(&self._reserved);
        Self::SIZE
    }

    #[inline]
    pub fn read_from(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let mut reserved = [0u8; 18];
        reserved.copy_from_slice(&buf[30..48]);
        
        Some(Self {
            start_ms: u64::from_le_bytes([
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]
            ]),
            end_ms: u64::from_le_bytes([
                buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]
            ]),
            total_distance_cm: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            total_steps: u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
            total_disruptions: u16::from_le_bytes([buf[24], buf[25]]),
            top_importance: buf[26],
            primary_transport: buf[27],
            movement_pct: buf[28],
            avg_disruption_conf: buf[29],
            _reserved: reserved,
        })
    }

    /// Get total distance in meters
    #[inline]
    pub fn total_distance_m(&self) -> f32 { self.total_distance_cm as f32 / 100.0 }
}

// ============================================================================
// WIRE WRITER (zero-allocation streaming)
// ============================================================================

/// Pre-allocated buffer writer for streaming wire messages.
/// 
/// Designed for ring buffer / shared memory scenarios where
/// the buffer is pre-allocated and reused.
pub struct WireWriter {
    /// Internal buffer
    buffer: Vec<u8>,
    /// Current write position
    position: usize,
    /// Sequence number for messages
    sequence: u64,
}

impl WireWriter {
    /// Create a new writer with given capacity.
    pub fn new(capacity: usize) -> Self {
        Self {
            buffer: vec![0u8; capacity],
            position: 0,
            sequence: 0,
        }
    }

    /// Reset writer position to beginning.
    #[inline]
    pub fn reset(&mut self) {
        self.position = 0;
    }

    /// Get current position.
    #[inline]
    pub fn position(&self) -> usize {
        self.position
    }

    /// Get buffer slice up to current position.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.buffer[..self.position]
    }

    /// Get raw buffer pointer for FFI.
    #[inline]
    pub fn as_ptr(&self) -> *const u8 {
        self.buffer.as_ptr()
    }

    /// Get sequence number.
    #[inline]
    pub fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Write a trajectory update.
    pub fn write_trajectory(&mut self, point: &TrajectoryPoint) -> bool {
        let needed = WireHeader::SIZE + WireTrajectory::SIZE;
        if self.position + needed > self.buffer.len() {
            return false;
        }

        let header = WireHeader::new(MessageType::TrajectoryUpdate, WireTrajectory::SIZE as u32);
        let traj = WireTrajectory::from_trajectory(point);

        self.position += header.write_to(&mut self.buffer[self.position..]);
        self.position += traj.write_to(&mut self.buffer[self.position..]);
        self.sequence += 1;
        true
    }

    /// Write a step event.
    pub fn write_step(&mut self, step: &StepEvent, cumulative_count: u32, current_heading_rad: f32) -> bool {
        let needed = WireHeader::SIZE + WireStep::SIZE;
        if self.position + needed > self.buffer.len() {
            return false;
        }

        let header = WireHeader::new(MessageType::StepEvent, WireStep::SIZE as u32);
        let wire_step = WireStep::from_step(step, cumulative_count, current_heading_rad);

        self.position += header.write_to(&mut self.buffer[self.position..]);
        self.position += wire_step.write_to(&mut self.buffer[self.position..]);
        self.sequence += 1;
        true
    }

    /// Write a disruption event.
    pub fn write_disruption(&mut self, disruption: &ContextDisruption, rank: u16) -> bool {
        let needed = WireHeader::SIZE + WireDisruption::SIZE;
        if self.position + needed > self.buffer.len() {
            return false;
        }

        let header = WireHeader::new(MessageType::DisruptionEvent, WireDisruption::SIZE as u32);
        let wire_disruption = WireDisruption::from_disruption(disruption, rank);

        self.position += header.write_to(&mut self.buffer[self.position..]);
        self.position += wire_disruption.write_to(&mut self.buffer[self.position..]);
        self.sequence += 1;
        true
    }

    /// Write a transport mode change.
    pub fn write_transport_change(
        &mut self, 
        timestamp_ms: u64, 
        from: TransportMode, 
        to: TransportMode, 
        confidence: f32
    ) -> bool {
        let needed = WireHeader::SIZE + WireTransport::SIZE;
        if self.position + needed > self.buffer.len() {
            return false;
        }

        let header = WireHeader::new(MessageType::TransportChange, WireTransport::SIZE as u32);
        let transport = WireTransport::new(timestamp_ms, from, to, confidence);

        self.position += header.write_to(&mut self.buffer[self.position..]);
        self.position += transport.write_to(&mut self.buffer[self.position..]);
        self.sequence += 1;
        true
    }

    /// Write session summary.
    pub fn write_session_summary(&mut self, summary: &WireSessionSummary) -> bool {
        let needed = WireHeader::SIZE + WireSessionSummary::SIZE;
        if self.position + needed > self.buffer.len() {
            return false;
        }

        let header = WireHeader::new(MessageType::SessionSummary, WireSessionSummary::SIZE as u32);

        self.position += header.write_to(&mut self.buffer[self.position..]);
        self.position += summary.write_to(&mut self.buffer[self.position..]);
        self.sequence += 1;
        true
    }
}

// ============================================================================
// WIRE READER (zero-copy parsing)
// ============================================================================

/// Zero-copy wire message reader.
pub struct WireReader<'a> {
    buffer: &'a [u8],
    position: usize,
}

impl<'a> WireReader<'a> {
    /// Create reader from buffer slice.
    pub fn new(buffer: &'a [u8]) -> Self {
        Self { buffer, position: 0 }
    }

    /// Check if more messages are available.
    #[inline]
    pub fn has_next(&self) -> bool {
        self.position + WireHeader::SIZE <= self.buffer.len()
    }

    /// Peek at next message type without advancing.
    pub fn peek_type(&self) -> Option<MessageType> {
        WireHeader::read_from(&self.buffer[self.position..])
            .and_then(|h| h.message_type())
    }

    /// Read next message. Returns (type, payload_slice).
    pub fn next_message(&mut self) -> Option<(MessageType, &'a [u8])> {
        let header = WireHeader::read_from(&self.buffer[self.position..])?;
        let msg_type = header.message_type()?;
        
        let payload_start = self.position + WireHeader::SIZE;
        let payload_end = payload_start + header.payload_len as usize;
        
        if payload_end > self.buffer.len() {
            return None;
        }

        self.position = payload_end;
        Some((msg_type, &self.buffer[payload_start..payload_end]))
    }

    /// Skip current message.
    pub fn skip(&mut self) -> bool {
        if let Some(header) = WireHeader::read_from(&self.buffer[self.position..]) {
            let new_pos = self.position + WireHeader::SIZE + header.payload_len as usize;
            if new_pos <= self.buffer.len() {
                self.position = new_pos;
                return true;
            }
        }
        false
    }

    /// Get remaining bytes.
    #[inline]
    pub fn remaining(&self) -> usize {
        self.buffer.len().saturating_sub(self.position)
    }
}

// ============================================================================
// HELPERS
// ============================================================================

#[inline]
fn transport_to_u8(mode: TransportMode) -> u8 {
    match mode {
        TransportMode::Unknown => 0,
        TransportMode::Stationary => 1,
        TransportMode::Walking => 2,
        TransportMode::Running => 3,
        TransportMode::Vehicle => 4,
        TransportMode::Cycling => 5,
        TransportMode::Stairs => 6,
        TransportMode::Elevator => 7,
    }
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_header_roundtrip() {
        let header = WireHeader::new(MessageType::TrajectoryUpdate, 32);
        let mut buf = [0u8; 8];
        header.write_to(&mut buf);
        
        let parsed = WireHeader::read_from(&buf).unwrap();
        // Use local copies to avoid packed struct reference issues
        let magic = parsed.magic;
        let payload_len = parsed.payload_len;
        assert_eq!(magic, WIRE_MAGIC);
        assert_eq!(parsed.message_type(), Some(MessageType::TrajectoryUpdate));
        assert_eq!(payload_len, 32);
    }

    #[test]
    fn test_trajectory_roundtrip() {
        let point = TrajectoryPoint {
            timestamp_ms: 123456789,
            x: 12.345,
            y: -67.89,
            z: 1.5,
            heading_rad: 1.57,
            velocity_mps: 1.2,
            confidence: 0.95,
            stance_phase: StancePhase::Swing,
            uncertainty_m: 0.5,
        };

        let wire = WireTrajectory::from_trajectory(&point);
        let mut buf = [0u8; 32];
        wire.write_to(&mut buf);
        
        let parsed = WireTrajectory::read_from(&buf).unwrap();
        assert_eq!(parsed.timestamp_ms, 123456789);
        assert!((parsed.x() - 12.345).abs() < 0.001);
        assert!((parsed.y() - (-67.89)).abs() < 0.001);
        assert_eq!(parsed.stance, 2); // Swing
    }

    #[test]
    fn test_wire_writer_capacity() {
        let mut writer = WireWriter::new(50); // Small buffer
        let point = TrajectoryPoint::new(1000);
        
        // First write should succeed (8 + 32 = 40 bytes)
        assert!(writer.write_trajectory(&point));
        assert_eq!(writer.position(), 40);
        
        // Second write should fail (would need 80 bytes)
        assert!(!writer.write_trajectory(&point));
    }

    #[test]
    fn test_wire_reader() {
        let mut writer = WireWriter::new(1024);
        let point = TrajectoryPoint::new(1000);
        writer.write_trajectory(&point);
        writer.write_trajectory(&point);

        let mut reader = WireReader::new(writer.as_bytes());
        
        let (msg_type, payload) = reader.next_message().unwrap();
        assert_eq!(msg_type, MessageType::TrajectoryUpdate);
        assert_eq!(payload.len(), 32);
        
        assert!(reader.has_next());
        let (msg_type2, _) = reader.next_message().unwrap();
        assert_eq!(msg_type2, MessageType::TrajectoryUpdate);
        
        assert!(!reader.has_next());
    }

    #[test]
    fn test_size_constants() {
        assert_eq!(WireHeader::SIZE, 8);
        assert_eq!(WireTrajectory::SIZE, 32);
        assert_eq!(WireStep::SIZE, 20);
        assert_eq!(WireDisruption::SIZE, 64);
        assert_eq!(WireTransport::SIZE, 12);
        assert_eq!(WireSessionSummary::SIZE, 48);
    }

    #[test]
    fn test_step_heading_encoding() {
        // Use actual StepEvent fields
        let step = StepEvent {
            timestamp_ms: 1000,
            stride_length_m: 0.75,
            step_frequency_hz: 2.0, // 2 steps/sec = 120 steps/min
            confidence: 0.9,
            peak_accel: 15.0,
        };
        
        let wire = WireStep::from_step(&step, 42, 0.0);
        assert_eq!(wire.stride_cm, 75);
        assert_eq!(wire.cadence_spm, 120);
        assert_eq!(wire.count(), 42);
        assert!((wire.stride_m() - 0.75).abs() < 0.01);
    }
}
