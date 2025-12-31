//! C FFI Bindings for React Native Integration
//!
//! This module exposes the IMU engine to mobile platforms via C ABI.
//! React Native apps can call these functions through native bridges.
//!
//! Memory Safety:
//! - All returned strings must be freed with `trace_free_string()`
//! - The engine instance must be freed with `trace_engine_destroy()`
//! - NULL checks are performed on all inputs
//!
//! Thread Safety:
//! - The engine is NOT thread-safe. Use a single thread or mutex.

use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::ptr;

use crate::pipeline::FullMovementPipeline;
use crate::context_disruption::ContextDisruptionDetector;
use crate::export::{SessionExportBuilder, StreamingExporter};
use crate::wire::WireWriter;
use crate::types::{ImuSample, TransportMode};

// ============================================================================
// OPAQUE HANDLE TYPES
// ============================================================================

/// Opaque handle to the Trace IMU Engine.
pub struct TraceEngine {
    pipeline: FullMovementPipeline,
    disruption_detector: ContextDisruptionDetector,
    session_builder: SessionExportBuilder,
    streaming_exporter: StreamingExporter,
    wire_writer: WireWriter,
    last_transport_mode: TransportMode,
    
    // Binary output buffer (primary - fast path)
    disruption_count: usize,
    
    // JSON cache (slow path - only for debug/CEBE-X batch)
    last_disruption_json: Option<CString>,
    last_trajectory_json: Option<CString>,
}

/// Result status codes.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceStatus {
    /// Operation succeeded.
    Ok = 0,
    /// Null pointer provided.
    NullPointer = 1,
    /// Invalid parameter value.
    InvalidParameter = 2,
    /// Engine not initialized.
    NotInitialized = 3,
    /// Internal error.
    InternalError = 4,
}

/// Output from a single sample processing.
#[repr(C)]
#[derive(Debug, Clone)]
pub struct TraceSampleOutput {
    /// Timestamp in milliseconds.
    pub timestamp_ms: u64,
    /// X position in meters (relative to start).
    pub position_x: f32,
    /// Y position in meters (relative to start).
    pub position_y: f32,
    /// Z position in meters (altitude change).
    pub position_z: f32,
    /// Heading in radians (0 = East, π/2 = North).
    pub heading_rad: f32,
    /// Speed in m/s.
    pub velocity_mps: f32,
    /// Transport mode (0=Unknown, 1=Stationary, 2=Walking, 3=Vehicle, 4=Stairs, 5=Elevator).
    pub transport_mode: i32,
    /// Step detected this sample (0 or 1).
    pub step_detected: i32,
    /// Number of disruptions detected this sample.
    pub disruption_count: i32,
    /// Confidence in the current estimate [0.0, 1.0].
    pub confidence: f32,
}

/// Configuration for the engine.
#[repr(C)]
#[derive(Debug, Clone)]
pub struct TraceConfig {
    /// IMU sampling rate in Hz (typically 50-200).
    pub sample_rate_hz: f32,
    /// Device ID string (null-terminated).
    pub device_id: *const c_char,
    /// Enable aggressive disruption detection.
    pub high_sensitivity: i32,
}

// ============================================================================
// ENGINE LIFECYCLE
// ============================================================================

/// Create a new Trace IMU Engine instance.
///
/// # Safety
/// - `config` must be a valid pointer to TraceConfig.
/// - `config.device_id` must be a valid null-terminated C string.
/// - The returned pointer must be freed with `trace_engine_destroy()`.
///
/// # Returns
/// - Pointer to TraceEngine on success.
/// - NULL on failure.
#[no_mangle]
pub unsafe extern "C" fn trace_engine_create(config: *const TraceConfig) -> *mut TraceEngine {
    if config.is_null() {
        return ptr::null_mut();
    }
    
    let config = &*config;
    
    // Parse device ID
    let device_id = if config.device_id.is_null() {
        "unknown".to_string()
    } else {
        match CStr::from_ptr(config.device_id).to_str() {
            Ok(s) => s.to_string(),
            Err(_) => "unknown".to_string(),
        }
    };
    
    // Create pipeline with sample rate
    let mut pipeline = FullMovementPipeline::new();
    if config.sample_rate_hz > 0.0 {
        // Pipeline uses sample rate internally
        pipeline.set_sample_rate(config.sample_rate_hz);
    }
    
    // Create disruption detector
    let disruption_detector = if config.high_sensitivity != 0 {
        let mut dc = crate::context_disruption::DisruptionConfig::default();
        dc.min_confidence = 0.4;
        dc.min_importance = 0.2;
        ContextDisruptionDetector::new(dc)
    } else {
        ContextDisruptionDetector::default_detector()
    };
    
    let engine = Box::new(TraceEngine {
        pipeline,
        disruption_detector,
        session_builder: SessionExportBuilder::new(&device_id),
        streaming_exporter: StreamingExporter::new(&device_id),
        wire_writer: WireWriter::new(8192), // 8KB buffer for streaming
        last_transport_mode: TransportMode::Unknown,
        disruption_count: 0,
        last_disruption_json: None,
        last_trajectory_json: None,
    });
    
    Box::into_raw(engine)
}

/// Destroy a Trace IMU Engine instance.
///
/// # Safety
/// - `engine` must be a valid pointer from `trace_engine_create()`.
/// - Must not be called more than once for the same pointer.
#[no_mangle]
pub unsafe extern "C" fn trace_engine_destroy(engine: *mut TraceEngine) {
    if !engine.is_null() {
        drop(Box::from_raw(engine));
    }
}

/// Reset the engine state.
///
/// # Safety
/// - `engine` must be a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn trace_engine_reset(engine: *mut TraceEngine) -> TraceStatus {
    if engine.is_null() {
        return TraceStatus::NullPointer;
    }
    
    let engine = &mut *engine;
    engine.pipeline.reset();
    engine.disruption_detector.reset();
    engine.last_disruption_json = None;
    engine.last_trajectory_json = None;
    
    TraceStatus::Ok
}

// ============================================================================
// SAMPLE PROCESSING
// ============================================================================

/// Process a single IMU sample.
///
/// # Safety
/// - `engine` must be a valid pointer.
/// - `output` must be a valid pointer to receive results.
///
/// # Parameters
/// - `timestamp_ms`: Sample timestamp in milliseconds.
/// - `accel_x/y/z`: Accelerometer readings in m/s² (body frame).
/// - `gyro_x/y/z`: Gyroscope readings in rad/s (body frame).
/// - `mag_x/y/z`: Magnetometer readings in μT (set all to 0.0 if unavailable).
#[no_mangle]
pub unsafe extern "C" fn trace_process_sample(
    engine: *mut TraceEngine,
    timestamp_ms: u64,
    accel_x: f32, accel_y: f32, accel_z: f32,
    gyro_x: f32, gyro_y: f32, gyro_z: f32,
    mag_x: f32, mag_y: f32, mag_z: f32,
    output: *mut TraceSampleOutput,
) -> TraceStatus {
    if engine.is_null() || output.is_null() {
        return TraceStatus::NullPointer;
    }
    
    let engine = &mut *engine;
    let output = &mut *output;
    
    // Create IMU sample - use arrays to match ImuSample's expected types
    let sample = ImuSample {
        timestamp_ms,
        accel: [accel_x, accel_y, accel_z],
        gyro: [gyro_x, gyro_y, gyro_z],
        mag: if mag_x == 0.0 && mag_y == 0.0 && mag_z == 0.0 {
            None
        } else {
            Some([mag_x, mag_y, mag_z])
        },
        baro: None,
    };
    
    // Process through pipeline
    let result = engine.pipeline.process(&sample);
    
    // Run disruption detection
    let step_event = result.step_event.as_ref();
    let micro_events: Vec<_> = result.micro_events.iter().cloned().collect();
    
    let disruptions = engine.disruption_detector.update(
        timestamp_ms,
        &result.trajectory,
        result.transport_mode,
        step_event,
        &micro_events,
    );
    
    // Update session builder
    engine.session_builder.add_trajectory_point(&result.trajectory);
    if result.step_event.is_some() {
        engine.session_builder.add_step();
    }
    for d in &disruptions {
        engine.session_builder.add_disruption(d.clone());
    }
    
    // Write to binary wire buffer (fast path - primary)
    engine.wire_writer.reset();
    engine.wire_writer.write_trajectory(&result.trajectory);
    if let Some(ref step) = result.step_event {
        // Pass current heading from trajectory for step event
        engine.wire_writer.write_step(step, engine.pipeline.step_count() as u32, result.trajectory.heading_rad);
    }
    for (i, d) in disruptions.iter().enumerate() {
        engine.wire_writer.write_disruption(d, (i + 1) as u16);
    }
    engine.disruption_count = disruptions.len();
    
    // Cache disruption JSON only if there are disruptions (slow path - for debug)
    if !disruptions.is_empty() {
        let json_array: Vec<String> = disruptions.iter().map(|d| d.to_json()).collect();
        let json = format!("[{}]", json_array.join(","));
        engine.last_disruption_json = CString::new(json).ok();
    } else {
        engine.last_disruption_json = None;
    }
    
    // Skip JSON trajectory - use binary instead
    engine.last_trajectory_json = None;
    
    // Fill output struct
    output.timestamp_ms = timestamp_ms;
    output.position_x = result.trajectory.x;
    output.position_y = result.trajectory.y;
    output.position_z = result.trajectory.z;
    output.heading_rad = result.trajectory.heading_rad;
    output.velocity_mps = result.trajectory.velocity_mps;
    output.transport_mode = match result.transport_mode {
        TransportMode::Unknown => 0,
        TransportMode::Stationary => 1,
        TransportMode::Walking => 2,
        TransportMode::Running => 2, // Treat running like walking for C API
        TransportMode::Vehicle => 3,
        TransportMode::Cycling => 3, // Treat cycling like vehicle for C API
        TransportMode::Stairs => 4,
        TransportMode::Elevator => 5,
    };
    output.step_detected = if result.step_event.is_some() { 1 } else { 0 };
    output.disruption_count = disruptions.len() as i32;
    output.confidence = result.trajectory.confidence;
    
    engine.last_transport_mode = result.transport_mode;
    
    TraceStatus::Ok
}

// ============================================================================
// BINARY WIRE OUTPUT (FAST PATH - PRODUCTION)
// ============================================================================

/// Get pointer to binary wire buffer from last sample.
/// This is the PRIMARY export method for production use.
///
/// # Safety
/// - `engine` must be a valid pointer.
/// - Returned pointer is valid until next `trace_process_sample()` call.
///
/// # Wire Format
/// See wire.rs for message format documentation.
/// Buffer contains: Header(8) + Trajectory(32) + [Step(20)] + [Disruption(64)*]
///
/// # Returns
/// - Pointer to binary buffer (do NOT free - owned by engine).
/// - NULL on error.
#[no_mangle]
pub unsafe extern "C" fn trace_get_wire_buffer(engine: *const TraceEngine) -> *const u8 {
    if engine.is_null() {
        return ptr::null();
    }
    let engine = &*engine;
    engine.wire_writer.as_ptr()
}

/// Get length of binary wire buffer from last sample.
#[no_mangle]
pub unsafe extern "C" fn trace_get_wire_buffer_len(engine: *const TraceEngine) -> u32 {
    if engine.is_null() {
        return 0;
    }
    let engine = &*engine;
    engine.wire_writer.position() as u32
}

/// Get number of disruptions in the wire buffer.
#[no_mangle]
pub unsafe extern "C" fn trace_get_wire_disruption_count(engine: *const TraceEngine) -> u32 {
    if engine.is_null() {
        return 0;
    }
    let engine = &*engine;
    engine.disruption_count as u32
}

// ============================================================================
// JSON OUTPUT RETRIEVAL (SLOW PATH - DEBUG/CEBE-X BATCH)
// ============================================================================

/// Get the JSON for disruptions detected in the last sample.
/// NOTE: This is the SLOW PATH. Prefer `trace_get_wire_buffer()` for production.
///
/// # Safety
/// - `engine` must be a valid pointer.
///
/// # Returns
/// - JSON string (must be freed with `trace_free_string()`).
/// - NULL if no disruptions.
#[no_mangle]
pub unsafe extern "C" fn trace_get_last_disruptions_json(
    engine: *const TraceEngine,
) -> *const c_char {
    if engine.is_null() {
        return ptr::null();
    }
    
    let engine = &*engine;
    match &engine.last_disruption_json {
        Some(json) => json.as_ptr(),
        None => ptr::null(),
    }
}

/// Get the JSON for the trajectory update from the last sample.
///
/// # Safety
/// - `engine` must be a valid pointer.
///
/// # Returns
/// - JSON string (must be freed with `trace_free_string()`).
/// - NULL on error.
#[no_mangle]
pub unsafe extern "C" fn trace_get_last_trajectory_json(
    engine: *const TraceEngine,
) -> *const c_char {
    if engine.is_null() {
        return ptr::null();
    }
    
    let engine = &*engine;
    match &engine.last_trajectory_json {
        Some(json) => json.as_ptr(),
        None => ptr::null(),
    }
}

/// Get the complete session export as JSON.
///
/// # Safety
/// - `engine` must be a valid pointer.
///
/// # Returns
/// - JSON string (MUST be freed with `trace_free_string()`).
/// - NULL on error.
///
/// # Note
/// This consumes the current session data. Call only at session end.
#[no_mangle]
pub unsafe extern "C" fn trace_export_session_json(
    engine: *mut TraceEngine,
) -> *mut c_char {
    if engine.is_null() {
        return ptr::null_mut();
    }
    
    let engine = &mut *engine;
    
    // Build session and get JSON
    let device_id = engine.streaming_exporter.device_id().to_string();
    let old_builder = std::mem::replace(
        &mut engine.session_builder,
        SessionExportBuilder::new(&device_id),
    );
    
    let session = old_builder.build();
    let json = session.to_json();
    
    match CString::new(json) {
        Ok(cstring) => cstring.into_raw(),
        Err(_) => ptr::null_mut(),
    }
}

/// Get only the disruptions as JSON (lightweight export for CEBE-X).
///
/// # Safety
/// - `engine` must be a valid pointer.
///
/// # Returns
/// - JSON string (MUST be freed with `trace_free_string()`).
#[no_mangle]
pub unsafe extern "C" fn trace_export_disruptions_json(
    engine: *mut TraceEngine,
) -> *mut c_char {
    if engine.is_null() {
        return ptr::null_mut();
    }
    
    let engine = &mut *engine;
    
    let device_id = engine.streaming_exporter.device_id().to_string();
    let old_builder = std::mem::replace(
        &mut engine.session_builder,
        SessionExportBuilder::new(&device_id),
    );
    
    let session = old_builder.build();
    let json = session.disruptions_only_json();
    
    match CString::new(json) {
        Ok(cstring) => cstring.into_raw(),
        Err(_) => ptr::null_mut(),
    }
}

/// Free a string returned by trace_export_* functions.
///
/// # Safety
/// - `ptr` must be a string returned by a trace_export_* function.
/// - Must not be called more than once for the same pointer.
#[no_mangle]
pub unsafe extern "C" fn trace_free_string(ptr: *mut c_char) {
    if !ptr.is_null() {
        drop(CString::from_raw(ptr));
    }
}

// ============================================================================
// STATUS QUERIES
// ============================================================================

/// Get the total number of steps detected.
#[no_mangle]
pub unsafe extern "C" fn trace_get_step_count(engine: *const TraceEngine) -> i32 {
    if engine.is_null() {
        return -1;
    }
    
    let engine = &*engine;
    engine.pipeline.step_count() as i32
}

/// Get the total number of disruptions detected.
#[no_mangle]
pub unsafe extern "C" fn trace_get_disruption_count(engine: *const TraceEngine) -> i32 {
    if engine.is_null() {
        return -1;
    }
    
    let engine = &*engine;
    engine.disruption_detector.total_disruptions() as i32
}

/// Get the current transport mode.
#[no_mangle]
pub unsafe extern "C" fn trace_get_transport_mode(engine: *const TraceEngine) -> i32 {
    if engine.is_null() {
        return -1;
    }
    
    let engine = &*engine;
    match engine.last_transport_mode {
        TransportMode::Unknown => 0,
        TransportMode::Stationary => 1,
        TransportMode::Walking | TransportMode::Running => 2,
        TransportMode::Vehicle | TransportMode::Cycling => 3,
        TransportMode::Stairs => 4,
        TransportMode::Elevator => 5,
    }
}

/// Get the streaming exporter sequence number.
#[no_mangle]
pub unsafe extern "C" fn trace_get_sequence(engine: *const TraceEngine) -> u64 {
    if engine.is_null() {
        return 0;
    }
    
    let engine = &*engine;
    engine.streaming_exporter.sequence()
}

// ============================================================================
// VERSION INFO
// ============================================================================

/// Get the library version string.
///
/// # Returns
/// - Static string, do NOT free.
#[no_mangle]
pub extern "C" fn trace_version() -> *const c_char {
    static VERSION: &[u8] = b"trace-imu-engine/1.0.0\0";
    VERSION.as_ptr() as *const c_char
}

/// Get feature flags (bitfield).
///
/// Bit 0: Step detection
/// Bit 1: Heading estimation
/// Bit 2: Transport mode classification
/// Bit 3: Micro-event detection
/// Bit 4: Context disruption detection
/// Bit 5: ZUPT integration
/// Bit 6: Magnetometer fusion
#[no_mangle]
pub extern "C" fn trace_features() -> u32 {
    0b1111111 // All features enabled
}


// ============================================================================
// HELPER IMPLEMENTATIONS
// ============================================================================

impl FullMovementPipeline {
    /// Set sample rate (placeholder - actual implementation may vary).
    pub fn set_sample_rate(&mut self, _rate_hz: f32) {
        // The pipeline adapts to sample rate automatically from timestamps
    }
}


// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_engine_lifecycle() {
        unsafe {
            let device_id = CString::new("test_device").unwrap();
            let config = TraceConfig {
                sample_rate_hz: 100.0,
                device_id: device_id.as_ptr(),
                high_sensitivity: 0,
            };
            
            let engine = trace_engine_create(&config);
            assert!(!engine.is_null());
            
            let status = trace_engine_reset(engine);
            assert_eq!(status, TraceStatus::Ok);
            
            trace_engine_destroy(engine);
        }
    }

    #[test]
    fn test_sample_processing() {
        unsafe {
            let device_id = CString::new("test").unwrap();
            let config = TraceConfig {
                sample_rate_hz: 100.0,
                device_id: device_id.as_ptr(),
                high_sensitivity: 0,
            };
            
            let engine = trace_engine_create(&config);
            let mut output = TraceSampleOutput {
                timestamp_ms: 0,
                position_x: 0.0,
                position_y: 0.0,
                position_z: 0.0,
                heading_rad: 0.0,
                velocity_mps: 0.0,
                transport_mode: 0,
                step_detected: 0,
                disruption_count: 0,
                confidence: 0.0,
            };
            
            let status = trace_process_sample(
                engine,
                1000,
                0.0, 0.0, 9.81, // accel
                0.0, 0.0, 0.0,  // gyro
                0.0, 0.0, 0.0,  // mag (none)
                &mut output,
            );
            
            assert_eq!(status, TraceStatus::Ok);
            assert_eq!(output.timestamp_ms, 1000);
            
            trace_engine_destroy(engine);
        }
    }

    #[test]
    fn test_version() {
        let version = trace_version();
        assert!(!version.is_null());
        
        unsafe {
            let version_str = CStr::from_ptr(version).to_str().unwrap();
            assert!(version_str.contains("trace-imu-engine"));
        }
    }

    #[test]
    fn test_features() {
        let features = trace_features();
        assert!(features & 0b0001 != 0); // Step detection
        assert!(features & 0b0010 != 0); // Heading
        assert!(features & 0b0100 != 0); // Transport mode
    }

    #[test]
    fn test_null_safety() {
        unsafe {
            let status = trace_engine_reset(ptr::null_mut());
            assert_eq!(status, TraceStatus::NullPointer);
            
            let status = trace_process_sample(
                ptr::null_mut(),
                0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                ptr::null_mut(),
            );
            assert_eq!(status, TraceStatus::NullPointer);
            
            assert_eq!(trace_get_step_count(ptr::null()), -1);
        }
    }
}
