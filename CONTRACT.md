# Sensing Engine — Contract

This is what the library does and doesn't do.

---

## Purpose

Takes raw accelerometer and gyroscope data, produces structured motion evidence. Motion only. No decisions, interpretation, location, or identity.

---

## What it provides

- **Orientation handling**: Phone position doesn't matter—pocket, hand, bag, all work
- **Transition detection**: Identifies stops, hesitations, resumes, turns
- **Temporal accuracy**: Timestamps and durations are reliable
- **Offline operation**: Works without network
- **Uncertainty handling**: Every result includes confidence and data quality scores
- **Clear failures**: Never silently guesses; marks unreliable data explicitly

---

## What it does NOT do

- Determine location or coordinates
- Build maps or recognize rooms
- Identify users or fingerprint devices
- Infer what you're doing or why
- Use GPS, WiFi, Bluetooth, cameras, or microphones
- Require cloud services
- Keep raw sensor data after processing
- Create motion signatures that could re-identify users

These are architectural limits, not temporary limitations.

---

## Privacy: Built in, not bolted on

- Raw sensor data doesn't stick around
- Data stays on device
- Processed evidence can't be reversed back to original motion
- No biometric signatures kept
- Motion information degrades over time

Even if other systems get compromised, the engine itself can't be used for tracking or surveillance.

---

## What you need to provide

- Accelerometer readings [x, y, z] in m/s²
- Gyroscope readings [x, y, z] in rad/s
- Timestamp in milliseconds

The engine assumes:
- Phone is being carried by a person
- Data is coming in at roughly 50Hz (tolerates 10-200Hz)
- Sensors are factory-calibrated (device bias removed)

When these assumptions break, the engine marks the data as degraded or invalid.

---

## What you get back

Each motion evidence window contains:

- Timeline of motion segments with confidence
- State transitions where something changed
- Duration bucketing (rough time grouping)
- Sensor quality assessment
- Validity flag (should you trust this?)
- Overall confidence score

Just facts. No interpretation or labels.

---

## Data quality levels

Every output is marked as one of:

| Level | Meaning |
|-------|---------|
| **Valid** | Use it normally |
| **Degraded** | Weak signal; use with reduced weight |
| **Invalid** | Too noisy; skip it |

The engine tells you which it is. No guessing.

---

## What the engine knows and doesn't know

**It knows:**
- Raw sensor measurements
- Motion physics
- That's it

**It doesn't know:**
- Zone Mapper
- Item ranking
- Rooms or environments
- User history
- Trace application logic

The engine connects to higher layers, never the other way around.

---

## Design approach

- Extract motion facts, let higher layers interpret
- When in doubt, surface extra signals (higher layers filter)
- Always mark uncertainty
- Clear failure modes, never silent failures

---

## Performance

These are fixed requirements:

- Processes each sample in microseconds (not milliseconds)
- Uses fixed memory amount (doesn't grow over time)
- Works as background process without draining resources
- Completely offline, no internet needed
- Safe on Android and iOS

Any feature that breaks these stays out.

---

## Testing

Every claim is verified:

- Full test coverage
- Tested against noisy data, sensor errors, edge cases
- Stress-tested with continuous input
- Proven to handle NaN and Infinity gracefully
- Zero silent failures in test suite

---

## Versioning

Any change to:
- What you pass in or get back (format)
- What we promise (guarantees)
- How privacy works (behavior)
- How failures happen (semantics)

= Version change. No surprise updates.

---

## This is binding

This contract is enforceable.

Any implementation that breaks this is wrong. Performance, features, or convenience don't override this.

---

## Summary

**Input:** Raw 6-axis sensor data  
**Output:** Motion facts with confidence levels  
**Constraint:** Cannot be used for surveillance, by design  
**Rule:** Extract truth, interpret later.

No shortcuts. No exceptions.
