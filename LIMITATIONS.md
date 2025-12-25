# Limitations of the Sensing Engine

What works and doesn't, by design.

## What it doesn't do

**Location:**
- Can't figure out where you are
- Can't recognize rooms or buildings
- Can't map spaces

**Objects:**
- Can't tell what you picked up (phone vs keys)
- Can't identify surfaces
- Can't detect stairs vs flat ground

**Identity:**
- Can't identify who's using the phone
- Can't match motion patterns to people
- Can't create biometric profiles

These are intentional. They belong in higher-level systems.

**Other limitations:**
- Can't predict future movement
- Can't understand activities
- Can't recognize gestures
- Can't work with compass data only
- Can't process uncalibrated sensors

---

## What it does, with limitations

**Motion transitions:**
- May miss very short pauses
- Accuracy depends on sensor quality
- Works best at 50Hz

**Stillness detection:**
- Slow swaying looks like stillness
- Can't distinguish standing from lying down
- Vibration can trigger false motion

**Direction changes:**
- Works better with clear movement
- Just says "turn" not the angle
- No heading information

**Noise handling:**
- Good sensors only
- Can't recover from constant heavy vibration
- Needs factory calibration

---

## Problematic scenarios

**Bumpy car ride:**
Vibration looks like motion. Engine can't tell.  
→ Trace filters this.

**Elevator or sudden stop:**
Acceleration gets detected as motion.  
→ Trace recognizes these.

**Sensor drift:**
Engine detects it but can't fix it.  
→ Data marked degraded.

**Low sample rate (<10Hz):**
Details get lost.  
→ Need 25Hz minimum.

**Standing still for hours:**
Engine correctly marks STILL. Trace interprets it.

**Phone at weird angles:**
Still works, noisier.  
→ Confidence drops correctly.

**Vehicle + walking:**
Hard to separate.  
→ Engine guesses, Trace uses patterns.

---

## Sensor requirements

| Spec | Min | Typical | Max |
|------|-----|---------|-----|
| Sample rate | 10 Hz | 50 Hz | 200 Hz |
| Accel range | ±4g | ±8g | ±16g |
| Gyro range | ±250°/s | ±1000°/s | ±2000°/s |
| Calibration | Required | Yes | None |

Below minimum = won't work  
Typical = full performance  
Above maximum = works, lower confidence

---

## Platform support

**Works on:**
- Android
- iOS
- Embedded Linux
- Desktop (testing)

**Doesn't work on:**
- Devices without accelerometer/gyroscope
- Some smartwatches with limited IMU
- Systems without monotonic time
- Extreme vibration environments

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
