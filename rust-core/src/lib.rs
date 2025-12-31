//! Trace IMU Movement Engine Library
//!
//! A comprehensive motion sensing engine that converts raw inertial sensor
//! streams into structured movement data including trajectories, steps,
//! transport modes, and micro-events.
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
//! - **Full IMU pipeline**: Step detection, stance detection, PDR trajectory,
//!   heading estimation, transport mode classification, and micro-event detection.
//!
//! # Core Modules
//!
//! - `orientation`: Madgwick AHRS filter for accurate orientation estimation
//! - `step_detection`: Detect steps using vertical acceleration peak analysis
//! - `stance_detection`: Classify stance/swing phases for ZUPT
//! - `heading`: Heading integration with gyro and magnetometer fusion
//! - `trajectory`: Pedestrian Dead Reckoning with ZUPT correction
//! - `vehicle_detection`: Classify transport modes (walking, vehicle, etc.)
//! - `micro_events`: Detect phone pickup, putdown, pocket interactions
//!
//! # Example
//!
//! ```ignore
//! use trace_sensing::pipeline::FullMovementPipeline;
//! use trace_sensing::types::ImuSample;
//!
//! let mut pipeline = FullMovementPipeline::new();
//!
//! // Process a raw IMU sample
//! let sample = ImuSample::new(1000, [0.0, 0.0, -9.81], [0.0, 0.0, 0.0]);
//! let output = pipeline.process_sample(&sample);
//!
//! // Access trajectory, steps, transport mode, etc.
//! println!("Position: ({:.2}, {:.2})", output.position.x, output.position.y);
//! println!("Step count: {}", output.step_count);
//! ```

pub mod types;
pub mod signal;
pub mod orientation;
pub mod segmentation;
pub mod transitions;
pub mod pipeline;
pub mod battery_optimized;

// Full IMU modules
pub mod step_detection;
pub mod stance_detection;
pub mod heading;
pub mod trajectory;
pub mod vehicle_detection;
pub mod micro_events;

// Context disruption and export (Imagine Cup differentiators)
pub mod context_disruption;
pub mod export;       // JSON for debugging/CEBE-X batch (slow path)
pub mod wire;         // Binary wire format for streaming (fast path)

// C FFI bindings for React Native
pub mod ffi;

#[cfg(test)]
mod integration_tests;

#[cfg(test)]
mod stress_tests;

// Re-export commonly used types
pub use types::{
    DurationBucket, ImuSample, MotionEvidenceWindow, MotionMode, MotionSegment, SensorHealth,
    TransitionCandidate, TransitionType, ValidityState,
    // New full IMU types
    StepEvent, StancePhase, TrajectoryPoint, TransportMode, MicroEvent, MicroEventCandidate,
    KinematicSignature, MovementEpisode,
};

pub use signal::{FilterConfig, GravityEstimator, SignalProcessor, ProcessedSample};

pub use orientation::{Quaternion, OrientationConfig, OrientationEstimator, MadgwickAHRS};

pub use segmentation::{SegmentationConfig, SegmentationEngine, SegmentClassification};

pub use transitions::{TransitionDetector, TransitionEvent, TransitionConfig};

pub use pipeline::{MotionEvidencePipeline, PipelineConfig, FullMovementPipeline, FullMovementConfig, MovementSampleOutput};

pub use battery_optimized::{BatteryOptimizedEngine, BatteryMetrics, SamplingMode};

// Re-export new full IMU modules
pub use step_detection::{StepDetector, StepDetectorConfig};
pub use stance_detection::{StanceDetector, StanceDetectorConfig, AdaptiveStanceDetector};
pub use heading::{HeadingEstimator, HeadingConfig};
pub use trajectory::{PdrEstimator, PdrConfig, IntegratedPdr};
pub use vehicle_detection::{TransportDetector, TransportDetectorConfig};
pub use micro_events::{MicroEventDetector, MicroEventConfig};

// Context disruption (the core Imagine Cup differentiator)
pub use context_disruption::{
    ContextDisruptionDetector, DisruptionConfig, ContextDisruption,
    DisruptionType, DisruptionFeatures,
};

// Binary wire format (production - fast path)
pub use wire::{
    WireWriter, WireReader, WireHeader, MessageType,
    WireTrajectory, WireStep, WireDisruption, WireTransport, WireSessionSummary,
    WIRE_MAGIC, WIRE_VERSION,
};

// JSON export (debugging/CEBE-X batch - slow path)
pub use export::{
    SessionExport, SessionExportBuilder, StreamingExporter, StreamingUpdate,
    TrajectoryExport, EpisodeExport, DisruptionExport, SessionSummary,
};

// FFI for React Native
pub use ffi::{
    TraceEngine, TraceConfig, TraceStatus, TraceSampleOutput,
    trace_engine_create, trace_engine_destroy, trace_engine_reset,
    trace_process_sample, trace_export_session_json, trace_free_string,
    trace_version, trace_features,
};

