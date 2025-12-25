# IMU Movement Engine — Architectural & Privacy Contract

**Version:** 1.1  
**Status:** Binding  
**Scope:** This contract governs the design, implementation, testing, and evolution of the IMU Movement Engine.

This document is **normative**, not descriptive.  
All implementations, optimizations, refactors, and extensions **must comply** with this contract.

If behavior conflicts with this document, the implementation is incorrect — regardless of performance gains.

---

## 1. Purpose

The IMU Movement Engine exists to:

> Convert raw inertial sensor streams into a privacy-preserving, structured timeline of motion changes (movement, pauses, transitions) suitable for downstream ranking and reasoning.

It is a **motion evidence kernel**, not a decision system, not a prediction system, and not an interpretation layer.

---

## 2. Core guarantees

When operating in a valid state, the engine guarantees:

1. **Orientation invariance**  
   Phone orientation (pocket, hand, bag, upside-down) does not affect motion evidence extraction.

2. **Transition preservation**  
   Stops, pauses, hesitations, resumes, and direction changes are consistently surfaced as candidate motion events.

3. **Temporal correctness**  
   Ordering and approximate duration of motion events are preserved within bounded error.

4. **Offline-first execution**  
   The engine functions fully without network connectivity or cloud services.

5. **Explicit uncertainty signaling**  
   Every output includes confidence, sensor health, and validity indicators.

6. **Fail-loud behavior**  
   When assumptions break or sensor quality degrades, the engine marks data as degraded or invalid instead of guessing.

---

## 3. Explicit non-goals

The IMU Movement Engine **must never** attempt to:

- determine absolute or relative location  
- calculate coordinates, trajectories, or navigation paths  
- build maps or spatial layouts  
- recognize objects, rooms, or surfaces  
- infer intent, emotion, cognition, or mental state  
- identify, fingerprint, or profile users  
- perform surveillance or continuous tracking  
- rely on GPS, Wi-Fi, Bluetooth, cameras, or microphones  
- require cloud or server-side inference  

If any of the above are required, they belong strictly to higher layers in Trace.

---

## 4. Ethical boundary

The engine is **architecturally incapable** of:

- real-time location tracking  
- user identification or profiling  
- surveillance or monitoring systems  

This limitation must hold **even if downstream systems are adversarial, compromised, or misused**.

No future extension may weaken this boundary.

---

## 5. Privacy guarantees (non-negotiable)

Privacy is enforced **by construction**, not by configuration or policy.

The engine guarantees:

- Raw IMU sensor streams never leave the device  
- Raw sensor data is not persisted beyond processing windows  
- Exported motion evidence is non-reversible  
- No stable gait, biometric, or identifying signatures are retained  
- Motion evidence is quantized and temporally decayed  

Downstream modules **cannot request increased precision** beyond what this engine emits.

---

## 6. Input contract

### Required inputs
- Accelerometer (x, y, z)  
- Gyroscope (x, y, z)  
- Monotonic timestamp  

### Optional inputs
- Step detector (if available)

### Assumptions
- Device is human-carried  
- Sampling rate is approximately stable  
- Device orientation may change arbitrarily  

When assumptions break, the engine degrades gracefully and signals reduced validity.

---

## 7. Output contract

The engine outputs **motion evidence windows**, never semantic labels or decisions.

Each window includes:
- motion segments  
- transition candidates  
- quantized duration ranges  
- coarse context mode  
- confidence score  
- sensor health state  
- validity state  
- schema version  

No output implies interpretation, intent, or meaning.

---

## 8. Validity and failure states

Every evidence window must be explicitly labeled as one of:

- `VALID_WINDOW`  
- `DEGRADED_WINDOW`  
- `INVALID_WINDOW`  

Rules:
- Invalid windows are dropped  
- Degraded windows are passed with reduced confidence  
- Silent failure is forbidden  

The system must prefer incomplete evidence over incorrect inference.

---

## 9. Architectural boundaries

- The IMU Movement Engine has **no knowledge of**:
  - Zone Mapper  
  - ranking or retrieval logic  
  - objects or environments  
  - user history or identity  

- Dependencies are strictly one-directional:
  - IMU Engine → higher layers  
  - never the reverse  

This boundary must not be violated.

---

## 10. Design philosophy

- Evidence first, interpretation later  
- Conservative signal extraction  
- Explicit uncertainty everywhere  
- Clear failure boundaries  

The engine is intentionally biased toward **over-surfacing candidate motion events** rather than missing real ones.

---

## 11. Performance constraints

The engine must satisfy the following constraints on mobile devices:

- fully offline operation  
- fixed memory footprint  
- O(1) processing per sample  
- negligible battery impact during background execution  
- safe embedding on Android and iOS  

Any feature that violates these constraints does not belong in this engine.

---

## 12. Contract verifiability

Every guarantee and non-goal in this contract must be **verifiable**.

This includes:
- unit tests for signal invariants  
- regression tests for non-goals  
- property-based tests for failure states  
- static and runtime checks enforcing privacy boundaries  

If a guarantee cannot be tested, it cannot be claimed.

---

## 13. Threat model assumption

This contract is designed assuming:
- downstream systems may be faulty or adversarial  
- data consumers may attempt misuse  
- external compromise is possible  

The engine must remain safe and privacy-preserving under these conditions.

---

## 14. Versioning rule

Any change to:
- guarantees  
- outputs  
- privacy behavior  
- failure semantics  

requires a **schema version bump**.

Silent behavioral changes are forbidden.

---

## 15. Enforcement

This contract is binding.

Any implementation, optimization, or extension that violates this contract is considered **incorrect**, regardless of accuracy, performance, or feature gains.