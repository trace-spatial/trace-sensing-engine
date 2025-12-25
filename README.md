# Trace Sensing Engine
Motion evidence kernel for interruption recovery

## The problem

You lose things when distracted. That distraction leaves a physical signature: stops, hesitations, turns, pauses. But motion alone solves this—no GPS indoors, cameras violate privacy, identity enables surveillance. Motion physics is pure.

## What this does

Converts raw phone IMU (accelerometer + gyroscope) into structured motion evidence windows. Shows when you stopped, hesitated, turned, resumed. No GPS. No network. No identity. Privacy by physics.

Input: raw 50Hz accelerometer + gyroscope  
Output: motion evidence windows with stops, hesitations, direction changes, durations, confidence, sensor health

## Core pipeline

```
Raw IMU → Signal Filtering → Orientation Normalization → Motion Segmentation → Transition Detection → Evidence Windows
```

1. **Signal Filtering**: Remove noise, separate gravity, assess sensor quality
2. **Orientation Normalization**: Phone position (pocket, hand, bag) doesn't matter
3. **Motion Segmentation**: Classify motion (Still/SteadyMotion/Turning/Transitional)
4. **Transition Detection**: Identify state changes with confidence
5. **Evidence Assembly**: Package results with validity, confidence, sensor health

## Example output

```rust
MotionEvidenceWindow {
    timestamp: 1000,
    duration_bucket: Medium,
    segments: [
        MotionSegment { mode: Walking, confidence: 0.92, duration_ms: 4200 },
        MotionSegment { mode: Still, confidence: 0.88, duration_ms: 800 }
    ],
    transitions: [
        TransitionCandidate { type: STOP, confidence: 0.85 },
        TransitionCandidate { type: RESUME, confidence: 0.79 }
    ],
    confidence: 0.86,
    sensor_health: Nominal,
    validity: Valid
}
```

## What it does NOT do

- Determine location or map spaces
- Recognize rooms, objects, surfaces
- Infer intent, emotion, mental state
- Identify or fingerprint users
- Use GPS, WiFi, Bluetooth, cameras, microphones
- Require cloud connectivity
- Store raw sensor data or gait signatures

These belong to higher layers. This engine preserves motion structure only.

## Key features

✅ **Privacy-first**: Raw sensor data never persists; evidence is non-reversible  
✅ **Offline**: Works fully without network; runs in background  
✅ **Battery efficient**: O(1) per-sample, fixed memory, <1% daily with adaptive sampling  
✅ **Orientation invariant**: Phone position doesn't affect results  
✅ **Fail-loud**: Never silently guesses; marks uncertainty explicitly  
✅ **Production ready**: 106 passing tests, stress-tested, proven NaN/Infinity handling

## Performance

- **Latency**: ~12 microseconds per sample
- **Memory**: ~8KB constant footprint, no dynamic allocation
- **Battery**: 0.02% baseline + 0.7% sensors = 0.72% daily, reduced to 0.24% with adaptive sampling
- **Target**: <1% daily total with WiFi batching

## How Trace uses this

```
Trace Application → Higher-level Reasoning → Sensing Engine → Raw IMU
```

Engine is the foundation. Higher layers add meaning and decisions. This layer only preserves motion truth.

## Getting started

- [USAGE.md](USAGE.md) — How to use the engine
- [CONTRACT.md](CONTRACT.md) — Privacy and architectural guarantees
- [LIMITATIONS.md](LIMITATIONS.md) — What's not supported
- [HOW_TRACE_USES_IT.md](HOW_TRACE_USES_IT.md) — How this integrates with Trace

## Status

106 tests passing. Production-ready Rust. Mobile-optimized. Zero external dependencies.

---

**Core principle**: Evidence first, interpretation later.  
Extract facts. Higher systems make decisions.