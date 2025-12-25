//! Trace IMU Movement Engine Library
//!
//! A privacy-first motion evidence kernel that converts raw inertial sensor
//! streams into structured motion evidence windows suitable for downstream
//! reasoning.
//!
//! # Design Philosophy
//!
//! This library is built on several core principles:
//!
//! - **Evidence first, interpretation later**: The engine extracts motion evidence
//!   without making decisions about what it means.
//! - **Fail-loud behavior**: Invalid data is marked as such, never silently passed through.
//! - **Privacy by construction**: Raw sensor data never leaves the device; exported
//!   evidence is non-reversible.
//! - **Battery awareness**: O(1) processing per sample with fixed memory footprint.
//!
//! # Example
//!
//! ```ignore
//! use trace_sensing::types::{ImuSample, MotionEvidenceWindow, MotionMode, SensorHealth};
//!
//! // Create a motion evidence window
//! let mut window = MotionEvidenceWindow::new(
//!     1,
//!     0,
//!     5000,
//!     MotionMode::SteadyMotion,
//!     SensorHealth::Nominal,
//!     1,
//! );
//!
//! // The window can now have segments and transitions added to it
//! ```

pub mod types;

// Re-export commonly used types
pub use types::{
    DurationBucket, ImuSample, MotionEvidenceWindow, MotionMode, MotionSegment, SensorHealth,
    TransitionCandidate, TransitionType, ValidityState,
};
