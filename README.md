# IMU Movement Engine
*A privacy-first motion evidence kernel for Trace*

---

## What this is

The IMU Movement Engine is the lowest-level component of Trace.

Its job is **not** to track users, **not** to determine location, and **not** to understand environments.

Its only responsibility is to convert raw phone motion sensors (accelerometer, gyroscope, step signals) into a **clean, structured timeline of how a person moved and paused over time**, especially around moments of interruption.

It preserves **how movement changed**, not **what happened**.

---

## Why this exists

People usually lose things right after they are interrupted.

That interruption leaves a physical signature in movement:
- a sudden stop  
- a short hesitation  
- a change in direction  
- an unusual pause before continuing  

Most systems ignore these signals or try to replace them with GPS, cameras, or identity-based tracking.

This engine is built on a different belief:

> Meaningful context can be preserved using only motion physics, without knowing who the user is, where they are, or what they interacted with.

By working purely on inertial data:
- the system runs fully offline  
- battery usage stays low  
- privacy is enforced by architecture, not policy  
- the engine works indoors and outdoors  
- the engine remains safe by default  

This makes it suitable as a foundation for **interruption-recovery systems** without becoming a surveillance tool.

---

## What the engine actually does

At a high level, the engine performs five steps:

1. **Signal cleaning**  
   Raw IMU streams are filtered to remove high-frequency noise and correct low-frequency drift so small hand movements or sensor bias do not appear as meaningful motion.

2. **Orientation normalization**  
   Motion is aligned to gravity so phone orientation (pocket, hand, bag) does not affect interpretation.

3. **Segmentation**  
   Continuous motion is broken into stable movement segments such as walking, standing, or slow transition.

4. **Transition detection**  
   Boundaries between segments are analyzed to identify pauses, stops, hesitations, and direction changes.

5. **Evidence window construction**  
   These segments and transitions are packaged into structured motion evidence windows with confidence and health metadata.

The engine does **not** decide what an action means.  
It only preserves motion structure so higher layers can reason safely.

---

## Example output (conceptual)

The engine does not output labels or predictions.  
It outputs **motion evidence windows**.

Example:

MotionEvidenceWindow (conceptual):

Segments:

walking (≈ 4s)

walking (≈ 3s)


Transitions:

sudden_stop (≈ 0.6s)

direction_change (≈ right turn)


Metadata:

duration_bucket: short_pause

confidence: 0.82

sensor_health: nominal

state: VALID_WINDOW


This window does **not** say what happened.  
It only preserves *how movement changed*.

---

## What this engine does NOT do

This engine explicitly does **not**:

- determine location or coordinates  
- build maps or spatial layouts  
- recognize rooms, objects, or surfaces  
- infer intent, emotion, or mental state  
- identify, fingerprint, or profile users  
- perform surveillance or continuous tracking  
- use GPS, Wi-Fi, Bluetooth, cameras, or microphones  
- require cloud connectivity or server-side processing  

If any of these are needed, they belong to higher layers in Trace, not here.

---

## Privacy by construction

Privacy is enforced by design, not added later.

- Raw IMU streams never leave the device  
- Raw sensor data is not persisted  
- Exported motion evidence is non-reversible  
- No stable gait or biometric signatures are stored  
- Evidence is quantized and temporally decayed  

**Even if downstream systems were compromised, this engine cannot be repurposed into a tracking or surveillance mechanism.**

This boundary is intentional and non-negotiable.

---

## Outputs: evidence, not decisions

The engine outputs **motion evidence windows**, never conclusions.

Each window includes:
- motion segments  
- transition candidates  
- quantized duration ranges  
- coarse context mode  
- confidence score  
- sensor health state  
- validity state  
- schema version  

Downstream systems are expected to **rank, reason, and adapt**, not blindly trust.

---

## Failure handling

The engine never fails silently.

Every output window is explicitly marked as one of:
- `VALID_WINDOW`
- `DEGRADED_WINDOW`
- `INVALID_WINDOW`

Invalid windows are dropped.  
Degraded windows are passed with reduced confidence.

The system always prefers incomplete evidence over incorrect inference.

---

## Architecture philosophy

- Evidence first, interpretation later  
- One-directional dependencies only  
- Clear failure boundaries  
- Minimal assumptions about the world  

This engine is intentionally conservative.  
It would rather surface extra candidate interruption points than miss real ones.

---

## Performance constraints

This engine is designed for continuous, background mobile execution:

- fully offline operation  
- fixed memory footprint  
- O(1) processing per sample  
- negligible battery impact  
- safe for Android and iOS  

Any feature that violates these constraints does not belong in this engine.

---

## How this fits into Trace

The IMU Movement Engine is the foundation layer of Trace.

It feeds structured motion evidence into higher components such as:
- Zone Mapper  
- interruption-aware ranking systems  
- retrieval and recovery logic  

Those layers add meaning.  
This engine only preserves motion truth.

---

## Contract

The architectural and privacy contract for this engine is defined in **CONTRACT.md**.

That document specifies:
- guarantees  
- non-goals  
- privacy boundaries  
- failure behavior  
- versioning rules  

All implementation work must follow that contract.

---

## Status

This repository is under active development.

Current focus:
- production-grade Rust core  
- mobile-safe execution  
- correctness and stability over feature count  

No shortcuts. No hype. Just a solid foundation.