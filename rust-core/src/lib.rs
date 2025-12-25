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
//! use trace_sensing::signal::SignalProcessor;
//!
//! let mut processor = SignalProcessor::new();
//!
//! // Process a raw IMU sample
//! let sample = ImuSample::new(1000, [0.0, 0.0, -9.81], [0.0, 0.0, 0.0]);
//! let processed = processor.process_sample(&sample);
//! ```

pub mod types;
pub mod signal;
pub mod orientation;
pub mod segmentation;
pub mod transitions;
pub mod pipeline;

#[cfg(test)]
mod integration_tests;

// Re-export commonly used types
pub use types::{
    DurationBucket, ImuSample, MotionEvidenceWindow, MotionMode, MotionSegment, SensorHealth,
    TransitionCandidate, TransitionType, ValidityState,
};

pub use signal::{FilterConfig, GravityEstimator, SignalProcessor, ProcessedSample};

pub use orientation::{Quaternion, OrientationConfig, OrientationEstimator};

pub use segmentation::{SegmentationConfig, SegmentationEngine, SegmentClassification};

pub use transitions::{TransitionDetector, TransitionEvent, TransitionConfig};

pub use pipeline::{MotionEvidencePipeline, PipelineConfig};

