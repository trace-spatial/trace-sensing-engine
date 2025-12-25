# Trace Sensing Engine

A motion detection library for Trace. Extracts motion evidence from your phone's accelerometer and gyroscope.

## Overview

When you lose something while distracted, that distraction leaves a trace in how you moved. You stopped, looked around, turned. This library picks up those signals from raw sensor data and turns them into structured evidence: when you stopped, when you hesitated, when you changed direction.

Why motion? GPS doesn't work indoors, cameras create privacy concerns, and using identity for tracking is risky. Motion is just physics—it tells you what happened without revealing who or where.

## How it works

The library processes raw sensor data through these stages:

```
Raw IMU → Filtering → Orientation → Classification → Transitions → Output
```

1. **Filter noise**: Clean up sensor readings and separate gravity from motion
2. **Handle orientation**: Phone position shouldn't matter—pocket, hand, or bag
3. **Classify motion**: Detect still, steady motion, turns, and transitions
4. **Find transitions**: Spot when motion state changes
5. **Package results**: Include confidence scores and sensor quality information

## What you get back

Each time the library processes sensor data, it returns a window of motion evidence:

```rust
MotionEvidenceWindow {
    timestamp: 1000,
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

## What it doesn't do

- Figure out location or build maps
- Recognize rooms, objects, or surfaces
- Infer what you're doing or why
- Identify who's using the phone
- Use GPS, WiFi, Bluetooth, cameras, or mics
- Require internet or cloud services
- Keep raw sensor data or motion signatures

Those responsibilities belong to higher-level systems. This library handles motion extraction only.

## Key features

- **Privacy**: Raw sensor data doesn't stick around; you can't reverse the evidence back to original motion
- **Offline**: Works without network, runs in the background
- **Efficient**: Fixed memory usage and constant processing time per sample
- **Orientation-independent**: Phone in pocket, hand, or bag—works the same
- **Explicit uncertainty**: Always includes confidence scores and sensor quality information
- **Thoroughly tested**: Full test coverage including edge cases and error scenarios

## Performance

- **Speed**: Processes each sample quickly enough for real-time use
- **Memory**: Uses a fixed amount of memory, doesn't grow over time
- **Power**: Designed to be efficient; pairs well with background processing

## Where this fits

The library is the foundation. Trace builds on top: higher-level systems take motion evidence and connect it to context, time, and location. This library just extracts the motion facts.

## Getting started

- [USAGE.md](USAGE.md) — How to use the engine
- [CONTRACT.md](CONTRACT.md) — Privacy and architectural guarantees
- [LIMITATIONS.md](LIMITATIONS.md) — What's not supported
- [HOW_TRACE_USES_IT.md](HOW_TRACE_USES_IT.md) — How this integrates with Trace

## Status

Fully tested, production-ready Rust library. Runs on iOS, Android, and desktop. No external dependencies.

Simple principle: extract the facts about motion, let other systems decide what they mean.