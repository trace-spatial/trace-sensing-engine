# Usage

## Rust API

### Basic Pipeline

```rust
use trace_sensing::{ImuSample, FullMovementPipeline, PipelineConfig};

let config = PipelineConfig::default();
let mut pipeline = FullMovementPipeline::new(config);

let sample = ImuSample::new(
    timestamp_ms,
    [ax, ay, az],
    [gx, gy, gz],
);

let result = pipeline.process(&sample);

// Trajectory point (always present)
println!("Position: ({}, {})", result.trajectory.x, result.trajectory.y);
println!("Heading: {}", result.trajectory.heading_rad);

// Step event (when detected)
if let Some(step) = result.step_event {
    println!("Step: {} m stride", step.stride_length_m);
}
```

### Battery-Optimized

```rust
use trace_sensing::{BatteryOptimizedEngine, PipelineConfig};

let mut engine = BatteryOptimizedEngine::new(PipelineConfig::default());

for sample in imu_stream {
    if let Some(window) = engine.process_sample(sample) {
        // Process window
    }
}

let metrics = engine.battery_metrics();
println!("Battery impact: {:.2}%/day", metrics.battery_percent());
```

### Context Disruption

```rust
use trace_sensing::ContextDisruptionDetector;

let mut detector = ContextDisruptionDetector::new();

// After pipeline.process():
let disruptions = detector.update(
    &result.trajectory,
    result.step_event.as_ref(),
    transport_mode,
    &micro_events,
);

for d in disruptions {
    println!("{}: conf={:.2} importance={:.2}", 
        d.disruption_type, d.confidence, d.importance);
}
```

## C FFI

### Initialization

```c
#include "trace_sensing.h"

TraceConfig config = {
    .sample_rate_hz = 100.0,
    .device_id = "device_001",
    .high_sensitivity = 0
};

TraceEngine* engine = trace_engine_create(&config);
if (!engine) {
    // Handle error
}
```

### Processing

```c
TraceSampleOutput output;

trace_process_sample(
    engine,
    timestamp_ms,
    accel_x, accel_y, accel_z,
    gyro_x, gyro_y, gyro_z,
    mag_x, mag_y, mag_z,  // 0.0 if unavailable
    &output
);

// output.has_step
// output.disruption_count
// output.transport_mode
```

### Wire Buffer Access

```c
const uint8_t* buf = trace_get_wire_buffer(engine);
uint32_t len = trace_get_wire_buffer_len(engine);

// Send to consumer
write(fd, buf, len);
```

### JSON Export

```c
// Per-sample disruptions
if (output.disruption_count > 0) {
    const char* json = trace_get_last_disruptions_json(engine);
    // json is owned by engine, valid until next process call
}

// Full session export
char* session = trace_export_session_json(engine);
// Caller owns this string
send_to_backend(session);
trace_free_string(session);
```

### Cleanup

```c
trace_engine_destroy(engine);
```

## Wire Format Parsing

```c
typedef struct {
    uint16_t magic;      // 0x5452 ("TR")
    uint8_t  msg_type;
    uint8_t  reserved;
    uint32_t payload_len;
} WireHeader;

const uint8_t* ptr = buf;
const uint8_t* end = buf + len;

while (ptr + 8 <= end) {
    WireHeader* hdr = (WireHeader*)ptr;
    if (hdr->magic != 0x5452) break;
    
    ptr += 8;  // Skip header
    
    switch (hdr->msg_type) {
        case 0x01:  // TrajectoryUpdate
            // 32 bytes: ts(8), x(4), y(4), z(4), heading(4), velocity(4), conf(1), stance(1), pad(2)
            break;
        case 0x02:  // StepEvent
            // 20 bytes
            break;
        case 0x03:  // DisruptionEvent
            // 64 bytes
            break;
    }
    
    ptr += hdr->payload_len;
}
```

## Configuration

```rust
let mut config = PipelineConfig::default();

// Window duration
config.window_duration_ms = 1000;

// Stillness threshold (m/s²)
config.segmentation_config.still_threshold = 0.15;

// Gravity filter (lower = more stable)
config.filter_config.gravity_filter_alpha = 0.01;
```

## Error Handling

Check validity state:

```rust
match window.validity_state {
    ValidityState::Valid => { /* use normally */ }
    ValidityState::Degraded => { /* reduced confidence */ }
    ValidityState::Invalid => { /* discard */ }
}
```

Check sensor health:

```rust
match window.sensor_health {
    SensorHealth::Nominal => { /* good */ }
    SensorHealth::Degraded => { /* noisy */ }
    SensorHealth::Critical => { /* unreliable */ }
}
```
