# How to Use the Trace Sensing Engine

## Basic setup

Add to your Rust project:

```toml
[dependencies]
trace_sensing = { path = "./trace-sensing-engine/rust-core" }
```

## Simple example: Feed raw IMU, get motion evidence

```rust
use trace_sensing::{
    ImuSample, MotionEvidencePipeline, PipelineConfig,
    BatteryOptimizedEngine
};

fn main() {
    // Create engine with default config
    let config = PipelineConfig::default();
    let mut pipeline = MotionEvidencePipeline::new(config);

    // Process raw sensor samples
    let sample = ImuSample::new(
        1000,                           // timestamp_ms
        [0.5, -0.2, -9.81],            // accel [x, y, z] m/s²
        [0.01, 0.02, -0.005]           // gyro [x, y, z] rad/s
    );

    if let Some(window) = pipeline.process_sample(&sample) {
        println!("Got motion evidence window:");
        println!("  Confidence: {}", window.confidence);
        println!("  Mode: {:?}", window.context_mode);
        println!("  Validity: {:?}", window.validity_state);
        println!("  Sensor health: {:?}", window.sensor_health);
    }
}
```

## Battery-optimized version (recommended)

The library includes adaptive sampling that adjusts processing based on motion state.

```rust
use trace_sensing::BatteryOptimizedEngine;
use trace_sensing::PipelineConfig;

fn main() {
    let config = PipelineConfig::default();
    let mut engine = BatteryOptimizedEngine::new(config);

    // Feed samples as they arrive from IMU
    for sample in imu_stream {
        // Skips samples intelligently based on motion state
        if let Some(window) = engine.process_sample(sample) {
            // Window ready - motion evidence extracted
            handle_motion_window(window);
        }
    }

    // Optional: check battery metrics
    let metrics = engine.battery_metrics();
    println!("Daily battery: {:.2}%", metrics.battery_percent());
}
```

## Batch processing (high-throughput)

Process multiple samples at once:

```rust
let samples = vec![
    ImuSample::new(1000, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0]),
    ImuSample::new(1020, [0.1, 0.0, 9.80], [0.0, 0.0, 0.0]),
    // ... more samples
];

let windows = engine.process_batch(&samples);
for window in windows {
    println!("Processed window at {}", window.timestamp);
}
```

## Interpreting motion evidence windows

```rust
pub struct MotionEvidenceWindow {
    pub timestamp: u64,                    // ms
    pub duration_bucket: DurationBucket,   // VeryShort/Short/Medium/Long/VeryLong
    pub segments: Vec<MotionSegment>,      // motion timeline
    pub transitions: Vec<TransitionCandidate>,  // state changes
    pub confidence: f32,                   // 0.0-1.0 overall confidence
    pub context_mode: MotionMode,          // Still/SteadyMotion/Turning/Transitional
    pub sensor_health: SensorHealth,       // Nominal/Degraded/Critical
    pub validity_state: ValidityState,     // Valid/Degraded/Invalid
}
```

### Key fields

**`context_mode`**: Current best estimate of motion type
- `Still`: Device not moving
- `SteadyMotion`: Consistent walking/motion
- `Turning`: Rotational motion detected
- `Transitional`: Unclear state (change in progress)

**`segments`**: Timeline of motion within this window
```rust
pub struct MotionSegment {
    pub mode: MotionMode,
    pub confidence: f32,        // How sure about this mode
    pub duration_ms: u32,       // How long this segment lasted
    pub start_sample: usize,
}
```

**`transitions`**: Boundaries between states where something changed
```rust
pub enum TransitionType {
    START,           // Started moving from still
    STOP,            // Stopped from movement
    TURN,            // Changed direction significantly
    HESITATION,      // Brief pause mid-motion
    ACTIVITY_SHIFT,  // Mode changed (walk→turn)
    DIRECTION_CHANGE,// Direction shifted
}

pub struct TransitionCandidate {
    pub transition_type: TransitionType,
    pub confidence: f32,
    pub timestamp_ms: u64,
}
```

**`sensor_health`**: Quality of input IMU
- `Nominal`: Good quality, trust the data
- `Degraded`: Noisy or inconsistent, confidence reduced
- `Critical`: Too much noise, data may be invalid

**`validity_state`**: Should this window be used?
- `Valid`: Full confidence, use normally
- `Degraded`: Partial confidence, use with caution
- `Invalid`: Don't use this window

## Real example: Interrupt detection

```rust
fn detect_interruption(window: &MotionEvidenceWindow) -> bool {
    // Only trust valid windows
    if window.validity_state != ValidityState::Valid {
        return false;
    }

    // Look for stops or hesitations
    let has_stop = window.transitions.iter()
        .any(|t| t.transition_type == TransitionType::STOP && t.confidence > 0.80);
    
    let has_hesitation = window.transitions.iter()
        .any(|t| t.transition_type == TransitionType::HESITATION && t.confidence > 0.75);

    has_stop || has_hesitation
}
```

## Configuration

Default config is 50Hz baseline with 1-second windows. Customize:

```rust
use trace_sensing::{
    PipelineConfig,
    signal::FilterConfig,
    segmentation::SegmentationConfig,
};

let mut config = PipelineConfig::default();

// Change window duration (milliseconds)
config.window_duration_ms = 2000;  // 2-second windows

// Adjust signal filtering (lower alpha = more stable)
config.filter_config.gravity_filter_alpha = 0.01;

// Customize motion classification thresholds
config.segmentation_config.still_threshold = 0.15;  // m/s²

let mut pipeline = MotionEvidencePipeline::new(config);
```

## Common patterns

### Pattern 1: Track motion state transitions

```rust
let mut last_mode = MotionMode::Still;

for window in windows {
    if window.context_mode != last_mode {
        println!("Mode changed: {:?} → {:?}", last_mode, window.context_mode);
        last_mode = window.context_mode;
    }
}
```

### Pattern 2: Detect brief interruptions

```rust
fn is_brief_interruption(window: &MotionEvidenceWindow) -> bool {
    matches!(window.duration_bucket, 
        DurationBucket::VeryShort | DurationBucket::Short)
    && window.transitions.iter()
        .any(|t| matches!(t.transition_type, 
            TransitionType::STOP | TransitionType::HESITATION))
    && window.confidence > 0.75
}
```

### Pattern 3: Confidence-based filtering

```rust
fn should_process(window: &MotionEvidenceWindow) -> bool {
    // Only act on high-confidence windows
    window.confidence > 0.80 
    && window.sensor_health == SensorHealth::Nominal
    && window.validity_state == ValidityState::Valid
}
```

## Battery monitoring

```rust
let metrics = engine.battery_metrics();

println!("Samples: {}", metrics.samples_processed);
println!("CPU time: {} µs", metrics.cpu_time_us);
println!("CPU %: {:.2}%", metrics.cpu_percent);
println!("Power: {:.1} mW", metrics.power_mw);
println!("Daily: {:.2} mWh", metrics.daily_mwh());
println!("Battery impact: {:.2}%", metrics.battery_percent());

// Check metrics
if metrics.cpu_percent < 5.0 {
    println!("Low CPU usage");
}
```

## Sampling modes (with adaptive engine)

The engine automatically adjusts sampling rates based on motion:

```
Still → 10 Hz
Steady → 25 Hz
Active → 50 Hz
Transition → 100 Hz
```

Mode switching happens automatically with hysteresis to avoid rapid changes.

## Error handling

Always check validity:

```rust
match window.validity_state {
    ValidityState::Valid => {
        // Trust this window fully
        process_motion_event(window);
    },
    ValidityState::Degraded => {
        // Reduce confidence or ignore
        if window.confidence > 0.85 {
            process_motion_event(window);
        }
    },
    ValidityState::Invalid => {
        // Skip completely
        log_skip(window);
    },
}
```

## Integration tips

1. **Feed continuously**: Engine expects steady 50Hz (or configured rate) IMU stream
2. **Don't buffer**: Process samples immediately, let engine handle windowing
3. **Trust confidence**: Use window.confidence to weight downstream decisions
4. **Fail gracefully**: Invalid windows are rare; handle them by ignoring
5. **Monitor battery**: Check metrics periodically, especially in long-running scenarios

## Performance baseline

- **Per-sample cost**: Microseconds (well below 1ms per window)
- **Memory**: Fixed small footprint (no dynamic allocation in critical paths)
- **Power**: Efficient background processing

---

See [LIMITATIONS.md](LIMITATIONS.md) for what the engine can't do.  
See [CONTRACT.md](CONTRACT.md) for guarantees and privacy boundaries.
