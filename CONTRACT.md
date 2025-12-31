# Contract

Interface contract for the sensing engine.

## Input

| Field | Type | Unit |
|-------|------|------|
| timestamp_ms | u64 | milliseconds |
| accel | [f32; 3] | m/s² |
| gyro | [f32; 3] | rad/s |
| mag (optional) | [f32; 3] | µT |

Assumptions:
- Sample rate: 10-200 Hz (optimal: 50-100 Hz)
- Sensors factory-calibrated
- Timestamps monotonic

## Output

### Binary Wire Format (Primary)

Pre-serialized buffer accessible via FFI:
- `trace_get_wire_buffer()` — pointer to buffer
- `trace_get_wire_buffer_len()` — current length

Message types:
- `0x01` TrajectoryUpdate (32 bytes)
- `0x02` StepEvent (20 bytes)
- `0x03` DisruptionEvent (64 bytes)
- `0x04` TransportChange (12 bytes)
- `0x05` SessionSummary (48 bytes)

### JSON Export (Debug)

Available via `trace_get_last_disruptions_json()` and `trace_export_session_json()`.

## Guarantees

- O(1) per-sample processing
- Fixed memory footprint
- No heap allocation on hot path (wire format)
- No network calls
- No filesystem access
- Thread-unsafe (caller must synchronize)

## Does Not

- Store raw sensor data
- Determine absolute location
- Identify users
- Use GPS/WiFi/Bluetooth
- Require network connectivity

## Versioning

Breaking changes increment major version:
- Input format changes
- Output format changes
- Guarantee changes

## Memory

Caller owns:
- TraceEngine instance (free with `trace_engine_destroy`)
- Strings from `trace_export_session_json` (free with `trace_free_string`)

Engine owns:
- Wire buffer (valid until next `trace_process_sample` call)
- Disruption JSON pointer (valid until next call)
