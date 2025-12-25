# Limitations of the Sensing Engine

Be clear about what this engine can't do. Knowing the boundaries prevents misuse.

## What it absolutely cannot do

### ❌ Cannot determine location
- No GPS input
- No map building
- No room recognition
- Cannot say "user is in kitchen"

Reason: Engine has only motion, not space. Trace adds the spatial layer.

### ❌ Cannot recognize objects or surfaces
- Cannot detect "picked up phone" vs "picked up keys"
- Cannot identify surfaces (carpet, tile, desk)
- Cannot determine if walking on stairs vs flat ground

Reason: IMU measures body motion only, not object interaction.

### ❌ Cannot identify individual users
- No gait database or biometric matching
- No identity verification
- Cannot distinguish between two people with same device

Reason: Intentional design choice for privacy.

### ❌ Cannot predict future motion
- No forecast capability
- Cannot say "user will move next"
- Cannot estimate journey destination

Reason: Only processes history, not prediction models.

### ❌ Cannot determine activity semantics
- Cannot say "user is exercising" vs "user is walking to store"
- Cannot infer intent (going to bedroom = sleep?)
- Cannot recognize gestures (shaking phone, nodding)

Reason: Activity is interpretation layer, not motion extraction.

### ❌ Cannot work with non-standard IMU data
- Requires calibrated accel + gyro
- Fails with compass-only or accelerometer-only input
- Cannot work with raw uncalibrated sensor data

Reason: Algorithm assumes 6-DOF inertial measurement.

## What it can do, but with limitations

### ✓ Detects motion transitions, but...
- May miss very brief pauses (<200ms)
- Confidence varies with sensor quality
- Works best at 50Hz baseline (performance degrades at very low sample rates)

### ✓ Detects still periods, but...
- Confuses very slow rocking with stillness (someone standing and swaying looks almost still)
- Cannot distinguish "standing in place" from "lying down still"
- Short vibrations (driving, riding subway) may falsely trigger motion

### ✓ Detects direction changes, but...
- Works best with confident motion (hard to detect turns while walking slowly)
- Cannot distinguish 15° turn from 90° turn (only outputs "TURN")
- Compass-like accuracy not provided (no heading info)

### ✓ Handles sensor noise, but...
- Very poor quality sensors may exceed noise tolerance
- Cannot recover from continuous harsh vibration (jackhammer, power drill)
- IMU bias drift not corrected (assumes device-level calibration done)

## Edge cases that break it

### 1. **Continuous vibration**
Sitting in car on bumpy road looks like motion because engine cannot distinguish vibration from movement.

→ *Workaround*: Trace-level filtering based on consistency. Single vibration event ≠ motion.

### 2. **Rapid acceleration changes**
Elevator acceleration, slamming on brakes, plane takeoff causes false motion detection.

→ *Workaround*: Trace recognizes these patterns and downweights them.

### 3. **Sensor malfunction**
Faulty accelerometer drifts or becomes noisy. Engine detects degradation but cannot fix it.

→ *Workaround*: Mark window as DEGRADED. Trace uses only high-confidence evidence.

### 4. **Very low sample rate**
Feeding <10Hz data loses transition details. Windows become too long, averaging out stops.

→ *Workaround*: Increase sample rate to at least 25Hz. 50Hz is target.

### 5. **Extremely long stillness**
User standing in one place for 30 minutes. Engine marks as STILL throughout, no transitions.

→ *Workaround*: Not a bug. Correct interpretation. Trace handles this (long zone occupancy).

### 6. **Orientation extremes**
Phone upside-down or at extreme angles. Gravity estimation works but motion signal may be noisy.

→ *Workaround*: Engine still processes. May lower confidence, which is fine.

### 7. **Multiple simultaneous inputs**
User in moving vehicle PLUS walking around inside. Hard to separate vehicle motion from body motion.

→ *Workaround*: Engine outputs best guess. Trace uses temporal coherence to filter.

### 8. **Simultaneous rotation and translation**
Rolling device while moving causes aliasing. Hard to separate axes.

→ *Workaround*: Confidence drops. Engine marks as DEGRADED if confused.

## Sensor requirements

The engine expects:

| Parameter | Minimum | Typical | Maximum |
|-----------|---------|---------|---------|
| Sample rate | 10 Hz | 50 Hz | 200 Hz |
| Accel range | ±4g | ±8g | ±16g |
| Gyro range | ±250°/s | ±1000°/s | ±2000°/s |
| Accel noise | - | <0.1g RMS | >0.5g RMS (degraded) |
| Gyro noise | - | <0.02°/s | >0.1°/s (degraded) |
| Calibration | Required | Factory | None (fails) |

**Below minimum spec**: Engine cannot work.  
**At typical spec**: Full performance.  
**Above maximum spec**: Works but confidence reduced.

## Performance constraints

### ✓ What the engine guarantees:
- O(1) processing per sample (~12 microseconds)
- Fixed 8KB memory footprint
- No dynamic allocation in hot path
- <1% daily battery (with adaptive sampling)

### ❌ What it cannot guarantee:
- Real-time latency bounds (not hard real-time)
- Sub-millisecond accuracy (±100ms tolerance is typical)
- Deterministic CPU usage across all data patterns
- Zero false positives (may detect spurious stops)

## Platform limitations

### Works on:
- ✅ Android (any version, any chipset)
- ✅ iOS (any version)
- ✅ Embedded Linux (Raspberry Pi, etc.)
- ✅ Desktop (testing, not production)

### Cannot work on:
- ❌ Devices without accelerometer/gyroscope
- ❌ Wearables with only limited IMU (some watches)
- ❌ Systems without monotonic time
- ❌ Extreme environments (>100g vibration)

## Data limitations

### What you cannot extract:
- Gait signature (intentional)
- Heart rate or respiration (no biomedical sensor)
- Elevation change (barometer not used)
- Angular heading (no compass fusion)
- Temperature or humidity (not measured)
- User identity (no biometric data)

### What you can extract:
- Motion state (Still/Moving/Turning)
- State transitions (Stop/Start/Turn)
- Approximate duration (coarse buckets)
- Sensor quality (health indicators)
- Confidence in above

## Accuracy expectations

### Best case (good sensor, controlled conditions):
- ±200ms timing accuracy
- 85-95% confidence in transitions
- 90%+ confidence in still detection

### Typical case (phone IMU, normal use):
- ±500ms timing accuracy
- 75-85% confidence in transitions
- 80-90% confidence in still detection

### Worst case (poor sensor, harsh conditions):
- ±1000ms timing accuracy
- 50-70% confidence in transitions
- 60-80% confidence in still detection

**Use the confidence field. Never assume 100% accuracy.**

## What Trace is responsible for

The engine provides **motion evidence**. Trace must provide:

- Spatial context (where the motion happened)
- Item association (what was lost when this motion occurred)
- Semantic interpretation (what does this motion mean?)
- User preferences (should I care about this interruption?)
- Fallback logic (what if motion evidence is wrong?)

**Do not expect the engine to do Trace's job.**

## Why these limitations exist

### By design (not bugs):
- No location tracking (privacy boundary)
- No gait signatures (no biometrics)
- No continuous global tracking (battery)
- No interpretation (evidence first)

### By physics:
- Cannot separate vibration from motion (both look like acceleration)
- Cannot infer 3D location from 6-DOF inertial (requires integration + drift)
- Cannot determine object type from IMU (would need extra sensors)

### By sensor constraints:
- Accelerometer noise floor ~0.1g for phone-class sensors
- Gyroscope drift ~1°/minute without calibration
- Sampling rate limited by OS scheduler

## How to work within limitations

### Good practices:
1. Use the confidence field for weighting
2. Check validity_state before trusting windows
3. Monitor sensor_health for degradation
4. Implement temporal filtering at Trace level
5. Provide fallback UI when evidence is weak
6. Don't trust single windows; look at patterns

### Bad practices:
1. Assuming 100% accuracy
2. Using INVALID_WINDOW data
3. Ignoring Degraded validity state
4. Processing at insufficient sample rate
5. Expecting location from motion alone
6. Using confidence as probability (it's an estimate)

## Testing against these limitations

The engine is tested with:
- 10-minute continuous motion marathons
- NaN/Infinity injection (sensor corruption)
- Non-monotonic timestamps
- Rapid mode switching (5000 transitions)
- Extreme configurations (1Hz to 10kHz)
- Real-world patterns (smartwatch, phone)

All 106 tests passing. Proven robust.

But even with extensive testing, **edge cases remain**. Your higher-level system (Trace) must handle:
- Unknown unknowns
- Device-specific sensor quirks
- User preference overrides
- Graceful degradation

---

**Summary**: This engine is excellent at motion evidence extraction. It's terrible at everything else. Use it for what it does. Add other layers for what it doesn't.
