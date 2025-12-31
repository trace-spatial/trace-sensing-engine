# trace-sensing-engine

IMU processing library for context disruption detection.

## Overview

Rust library that processes accelerometer and gyroscope data to detect moments when user attention may have shifted. Outputs trajectory, step events, and disruption signals.

## Modules

```
rust-core/src/
├── lib.rs                    # Public API
├── types.rs                  # Data structures
├── pipeline.rs               # Main processing pipeline
├── orientation.rs            # Madgwick AHRS
├── step_detection.rs         # Step detection
├── stance_detection.rs       # Stance/swing phase
├── heading.rs                # Heading estimation
├── trajectory.rs             # PDR (pedestrian dead reckoning)
├── vehicle_detection.rs      # Transport mode detection
├── micro_events.rs           # Phone interaction detection
├── context_disruption.rs     # Disruption detection
├── wire.rs                   # Binary wire format
├── export.rs                 # JSON export
└── ffi.rs                    # C FFI bindings
```

## Disruption Types

| Type | Weight |
|------|--------|
| PhonePlacement | 0.9 |
| TransportTransition | 0.8 |
| SearchingBehavior | 0.75 |
| AbruptHalt | 0.7 |
| EnvironmentTransition | 0.65 |
| ExtendedStillness | 0.6 |
| ContextResumption | 0.6 |
| Hesitation | 0.5 |

## Wire Format

Binary serialization for downstream consumers:

```
Header (8 bytes):
┌──────────┬────────┬──────────┬──────────────┐
│ Magic 2B │ Type 1B│ Rsv 1B   │ PayloadLen 4B│
└──────────┴────────┴──────────┴──────────────┘

Messages:
- TrajectoryUpdate: 32 bytes
- StepEvent: 20 bytes
- DisruptionEvent: 64 bytes
- TransportChange: 12 bytes
- SessionSummary: 48 bytes
```

Little-endian, 4-byte aligned.

## FFI

```c
TraceConfig config = {
    .sample_rate_hz = 100.0,
    .device_id = "device_001"
};
TraceEngine* engine = trace_engine_create(&config);

TraceSampleOutput output;
trace_process_sample(engine, timestamp_ms,
    ax, ay, az, gx, gy, gz, mx, my, mz, &output);

// Binary output (primary path)
const uint8_t* buf = trace_get_wire_buffer(engine);
uint32_t len = trace_get_wire_buffer_len(engine);

// JSON output (debug path)
const char* json = trace_get_last_disruptions_json(engine);

trace_engine_destroy(engine);
```

## Build

```bash
cd rust-core
cargo test
cargo build --release
```

Cross-compile:
```bash
cargo build --release --target aarch64-apple-ios
cargo build --release --target aarch64-linux-android
```

## Tests

```
190 passing
```

## Related Docs

- [CONTRACT.md](CONTRACT.md) — interface contract
- [USAGE.md](USAGE.md) — API reference
- [LIMITATIONS.md](LIMITATIONS.md) — constraints
