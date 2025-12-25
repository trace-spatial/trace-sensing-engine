# Sensing Engine — Contract

**Version:** 1.1  
**Status:** Binding. All work must comply.

---

## Purpose

Convert raw IMU streams → privacy-preserving motion evidence.

Motion only. No decisions. No interpretation. No location. No identity.

---

## What it guarantees

✅ **Orientation invariant**: Phone position doesn't matter (pocket, hand, bag, upside-down all work)  
✅ **Transition detection**: Stops, hesitations, resumes, turns consistently detected  
✅ **Temporal accuracy**: ±500ms ordering and duration (good enough for interruption detection)  
✅ **Offline always**: Zero network dependency  
✅ **Explicit uncertainty**: Every output includes confidence, health, validity  
✅ **Fail-loud**: Never guesses. Marks invalid data explicitly.

---

## What it MUST NOT do

❌ Determine location or coordinates  
❌ Build maps or recognize rooms  
❌ Identify users or fingerprint devices  
❌ Infer intent or mental state  
❌ Use GPS, WiFi, Bluetooth, cameras, microphones  
❌ Require cloud connectivity  
❌ Store raw sensor data persistently  
❌ Create reversible gait signatures  

**These are architectural boundaries. Not negotiable.**

---

## Privacy (by design, not policy)

- Raw IMU never persists
- Sensor data never leaves device
- Exported evidence is non-reversible
- No biometric signatures retained
- Motion quantized and temporally decayed

**Even if downstream systems are compromised, the engine cannot enable surveillance.**

---

## Inputs required

- Accelerometer [x, y, z] in m/s²
- Gyroscope [x, y, z] in rad/s
- Monotonic timestamp in ms

Assumptions:
- Device is human-carried
- ~50Hz baseline (10-200Hz tolerated)
- Calibrated sensors (device bias removed)

When assumptions break → validity reduced, data marked DEGRADED/INVALID.

---

## Outputs: evidence, not decisions

Motion evidence windows containing:
- Segments (motion timeline)
- Transitions (state changes with confidence)
- Duration bucket (coarse time)
- Sensor health (quality assessment)
- Validity state (trust this window?)
- Confidence score

**No labels. No interpretation. Only facts.**

---

## Validity states

Every window is explicitly one of:

| State | Meaning | Action |
|-------|---------|--------|
| **Valid** | Full confidence. Use normally. | Process it. |
| **Degraded** | Partial confidence. Weak signal. | Use with caution. Reduce ranking weight. |
| **Invalid** | Too much noise/corruption. | Discard. Use fallback. |

**Silent failure forbidden.**

---

## Architecture boundaries

The engine knows about:
- Raw IMU input
- Motion physics
- Nothing else

The engine does NOT know about:
- Zone Mapper
- Item ranking
- Environments or rooms
- User history or identity
- Trace application logic

Dependencies: One-directional only (engine → higher layers, never reverse).

---

## Design philosophy

- Evidence first, interpretation later
- Conservative (surface extra candidates rather than miss real ones)
- Explicit uncertainty everywhere
- Clear failure modes

The engine is intentionally biased toward **over-reporting** motion transitions. Higher layers filter.

---

## Performance constraints (non-negotiable)

- O(1) per sample (~12 microseconds)
- Fixed 8KB memory, no dynamic allocation
- <1% daily battery
- Fully offline
- Safe on Android/iOS

Any feature breaking these constraints does not belong here.

---

## Testing

Every guarantee must be verifiable.

Current state:
- 106 passing tests
- Stress-tested: 10-minute marathons, NaN/Infinity, rapid transitions
- Property-tested: confidence bounds, validity states
- Proven: zero silent failures

---

## Versioning

Any change to:
- Output format
- Guarantees
- Privacy behavior
- Failure semantics

= **schema version bump**

No silent changes.

---

## Enforcement

This contract is **binding**.

Any implementation violating this contract is incorrect. Period.

Performance, accuracy, or features do not override this contract.

---

## Summary

**In**: Raw 6-DOF inertial data  
**Out**: Motion evidence with confidence and validity  
**Constraint**: Zero surveillance capability by design  
**Philosophy**: Extract truth. Higher systems interpret.  

No shortcuts. No exceptions.
