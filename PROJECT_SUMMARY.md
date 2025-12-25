# Trace Sensing Engine — Project Status

Library for extracting motion evidence from phone sensors. Fully tested, ready to integrate.

---

## What's included

**Library (Rust)**
- trace_sensing — motion detection kernel
- Zero external dependencies
- Tested and working

**Documentation**
- README — what it does
- USAGE — how to use it  
- CONTRACT — what it promises
- LIMITATIONS — what it can't do
- HOW_TRACE_USES_IT — how Trace integrates
- LIBRARY_SETUP — integration patterns
- OFFLINE_INTEGRATION — clarifies local operation
- COMPONENT_INTEGRATION — code examples for Zone Mapper, CEBE-X, Confidential Computing

**Examples**
- basic_usage — simple demo
- battery_monitoring — adaptive sampling showcase

---

## Core capabilities

Takes raw accelerometer + gyroscope data, produces structured motion windows showing:
- When motion started/stopped
- When direction changed
- Confidence levels
- Sensor quality

Works offline. Runs in background. No network calls.

## Technical details

- **Language**: Rust 2021
- **Platform**: iOS, Android, desktop, embedded
- **Test coverage**: Full (edge cases, error scenarios, load testing)
- **Compilation**: Fast
- **Dependencies**: None (pure Rust)

---

## How it works

Raw sensor input → Filter and normalize → Classify motion → Detect transitions → Output evidence

Each step is straightforward and focused.

## Common questions

**Will it use HTTP?**  
No. It's a library that runs in your app process. Local callbacks only.

**How does it work with Zone Mapper?**  
Zone Mapper listens to motion events. When the engine detects transitions, Zone Mapper can trigger environmental scans.

**How does it work with CEBE-X?**  
CEBE-X takes motion features from the engine and feeds them to its ranking model. Local processing.

**How does it work with Confidential Computing?**  
Optional encrypted trace storage. Not from the engine itself, but downstream systems can use engine output.

## Architecture

Data flow from sensors to higher systems:

```
IMU (phone sensors)
    ↓
Sensing Engine Library (this)
    ↓
Motion Events (local callbacks)
    ├→ Zone Mapper
    ├→ CEBE-X Engine  
    ├→ Local storage
    └→ Optional: Encrypted trace
```

Everything runs on device. No cloud services. No HTTP from the library.

---

## Implementation

- Core modules: filtering, orientation, classification, transition detection
- Optional: adaptive sampling for power efficiency
- All code is straightforward Rust with no magic

---

## Next steps

1. Create native bridges (Swift/Kotlin)
2. Stream IMU data into library
3. Listen to motion event callbacks
4. Integrate with Zone Mapper / CEBE-X

Details are in the documentation files.

---

## What you get

- Structured motion evidence (motion windows)
- Confidence scores on each piece
- Data quality flags
- No interpretation, no location, no identity

---

## Quick Start

### 1. Add to your Rust project
```toml
[dependencies]
trace_sensing = { path = "./trace-sensing-engine/rust-core" }
```

### 2. Use in code
```rust
use trace_sensing::{ImuSample, BatteryOptimizedEngine, PipelineConfig};

let mut engine = BatteryOptimizedEngine::new(PipelineConfig::default());

for (accel, gyro, timestamp) in imu_stream {
    let sample = ImuSample::new(timestamp, accel, gyro);
    if let Some(window) = engine.process_sample(sample) {
        handle_motion_window(window);
    }
}
```

### 3. Wire into Zone Mapper / CEBE-X
See COMPONENT_INTEGRATION.md for complete examples.

---

## Next Steps

1. **Create native bridges** (Swift/Kotlin FFI to link Rust library)
2. **Integrate Zone Mapper** (implement MotionListener trait)
3. **Integrate CEBE-X** (feed motion features to ONNX model)
4. **Test on real devices** (battery, latency, accuracy)
5. **Pilot with users** (validate object search effectiveness)

## Key guarantees

- Runs offline, no network
- Privacy by design (data doesn't persist)
- Efficient background processing
- Explicit about confidence and data quality
- Thoroughly tested

These aren't claims—they're architectural requirements.

---

## Getting started

1. Read [README.md](README.md) to understand what it does
2. Look at [USAGE.md](USAGE.md) for code examples
3. Check [LIMITATIONS.md](LIMITATIONS.md) for what it can't do
4. Read [COMPONENT_INTEGRATION.md](COMPONENT_INTEGRATION.md) for how to integrate with Trace components

---

## Support

Documentation covers:
- Technical usage (USAGE.md)
- Design decisions (CONTRACT.md)  
- Edge cases and limitations (LIMITATIONS.md)
- Integration patterns (COMPONENT_INTEGRATION.md)
- Offline operation (OFFLINE_INTEGRATION.md)
