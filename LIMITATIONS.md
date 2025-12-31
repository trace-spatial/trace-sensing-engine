# Limitations

## Does Not Provide

- Absolute position or coordinates
- Room/building recognition
- User identification
- Gait biometrics
- Compass heading (gyro-only heading estimation)

These are architectural limits.

## Sensor Requirements

| Parameter | Minimum | Recommended |
|-----------|---------|-------------|
| Sample rate | 10 Hz | 50-100 Hz |
| Accel range | ±4g | ±8g |
| Gyro range | ±250°/s | ±1000°/s |
| Calibration | Required | Factory |

Below minimum: unreliable output.

## Known Constraints

### Vibration

Continuous vibration (vehicle, machinery) appears as motion. Engine cannot distinguish.

Mitigation: Downstream filtering based on transport mode.

### Drift

Gyroscope drift accumulates over time. Heading estimates degrade.

Mitigation: ZUPT during stance phase, magnetometer fusion when available.

### Orientation

Works in any phone orientation. Accuracy slightly reduced at extreme angles.

### Sample Gaps

Missing samples degrade accuracy. Timestamp gaps > 100ms reduce confidence.

### Stationary Detection

Slow swaying may register as stationary. Standing vs lying down not distinguishable.

## Accuracy

| Condition | Timing | Confidence |
|-----------|--------|------------|
| Good sensor, controlled | ±200ms | 85-95% |
| Phone IMU, normal use | ±500ms | 75-85% |
| Poor sensor, harsh | ±1s | 50-70% |

Use the confidence field. Do not assume 100% accuracy.

## Platform

Tested:
- Android (aarch64, armv7)
- iOS (aarch64)
- Linux (x86_64, aarch64)
- Windows (x86_64)

Not tested:
- WebAssembly
- Embedded RTOS

## Thread Safety

Engine is not thread-safe. Caller must synchronize access.

## Memory

Fixed allocation after initialization. No dynamic allocation on hot path (wire format). JSON export allocates.
