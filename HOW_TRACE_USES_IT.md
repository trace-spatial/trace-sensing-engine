# How Trace Uses the Sensing Engine

## Architecture

Trace is designed as layers:

```
┌─────────────────────────────────────────────────┐
│  Trace Application                              │
│  (User interface, retrieval, item ranking)      │
├─────────────────────────────────────────────────┤
│  Higher-Level Reasoning                         │
│  (Zone mapping, context inference, recovery)    │
├─────────────────────────────────────────────────┤
│  Trace Sensing Engine (THIS)                    │
│  (Motion evidence extraction from IMU)          │
├─────────────────────────────────────────────────┤
│  Operating System                               │
│  (Raw accelerometer + gyroscope from sensors)   │
└─────────────────────────────────────────────────┘
```

The sensing engine is the **foundation**. It doesn't make decisions. It extracts motion facts. Higher layers decide what to do with those facts.

## Why this separation?

### The engine's responsibility:
- Convert raw IMU → motion evidence
- Detect stops, hesitations, turns, pauses
- Mark confidence and sensor quality
- Run offline, use minimal battery
- Never lose data integrity or privacy

### Trace's responsibility:
- Connect motion evidence to items
- Build spatial/temporal context
- Rank items by interruption likelihood
- Handle user preferences and settings
- Decide retrieval strategy
- Own the UX

This separation is intentional. If the engine fails or is wrong, the whole system doesn't collapse. Trace can:
- Use last-known position
- Ask the user
- Use alternative signals
- Gracefully degrade

The engine just provides its best effort.

## Data flow

### Real-time flow

```
Phone Sensors (50Hz)
    ↓
Sensing Engine (continuous processing)
    ↓
Motion Evidence Windows (every ~1 second)
    ↓
Trace Higher-Level Processing
    ↓
User Interface / Item Ranking
```

### What Trace receives

Every 1-2 seconds, Trace gets a `MotionEvidenceWindow`:

```rust
{
    timestamp: 1702000000000,
    duration_bucket: Medium,
    confidence: 0.86,
    validity: Valid,
    sensor_health: Nominal,
    context_mode: Turning,
    segments: [
        { mode: Walking, confidence: 0.92, duration_ms: 4200 },
        { mode: Turning, confidence: 0.88, duration_ms: 800 },
    ],
    transitions: [
        { type: TURN, confidence: 0.85, timestamp: 1702000000800 },
    ],
}
```

This says: "Over the last ~1 second, user was mostly walking, then turned. Confidence in the turn is 85%."

It does NOT say: "User turned toward their desk" or "User walked into the kitchen."

That's Trace's job.

## Integration points

### 1. Continuous motion tracking

**What Trace does:**
- Collect windows continuously in background
- Track motion_mode over time (build state machine)
- Build timeline of where user's body was

**From the engine:**
- Regular motion evidence windows
- Confidence scores per mode
- Sensor health indicators

```rust
// Pseudocode - Trace's logic, not the engine's
while let Some(window) = engine.process_sample(sample) {
    trace_db.insert_motion_window(window);
    
    // Update timeline
    timeline.append(MotionEvent {
        timestamp: window.timestamp,
        mode: window.context_mode,
        confidence: window.confidence,
    });
}
```

### 2. Interruption detection

**What Trace does:**
- Recognize stops, pauses, hesitations
- Correlate with lost items (time-based search)
- Estimate "when user put item down"

**From the engine:**
- Transition candidates with type and confidence
- Duration buckets (was pause brief or long?)
- Segment boundaries

```rust
// Pseudocode - Trace's logic
for window in motion_windows {
    for transition in window.transitions {
        if transition.type == TransitionType::STOP {
            // This might be when user put item down
            search_backwards(transition.timestamp);
        }
    }
}
```

### 3. Zone/room inference

**What Trace does:**
- Map motion patterns to locations (still zones)
- Build room models from movement signatures
- Estimate where user spends time

**From the engine:**
- Duration of still periods
- Transitions between still and motion
- Motion mode sequences

```rust
// Pseudocode - Trace's logic
let zone = {
    still_duration: 45_000,      // 45 seconds of stillness
    boundary_transitions: [
        TransitionType::STOP,    // Entered this zone
        TransitionType::START,   // Left this zone
    ],
};
```

### 4. Recovery ranking

**What Trace does:**
- When user searches "where's my keys?"
- Rank candidate locations by motion evidence
- Pick most likely spot to search first

**From the engine:**
- Motion state at different times
- Confidence in those states
- Transition sequences

```rust
// Pseudocode - Trace's logic
// Keys were lost around 2:30 PM
// What was the motion evidence then?

let window_at_loss_time = db.query_motion(loss_time);

if window_at_loss_time.confidence > 0.80
   && matches!(window.context_mode, MotionMode::Still) {
    // High confidence user was still - likely in a zone
    search_still_zones(loss_time);
} else {
    // User was moving - expand search
    search_all_zones(loss_time);
}
```

## Privacy model

### What the engine preserves:
- Motion structure (how body moved)
- Timing and sequencing
- Transitions and pauses

### What the engine throws away:
- Raw accelerometer/gyroscope readings
- Specific acceleration values
- Detailed orientation
- Anything that could reconstruct gait

### What Trace adds on top:
- Spatial context ("user was in kitchen")
- Semantic meaning ("user picked something up")
- Item associations
- User history

Even if Trace's database is compromised, the raw sensor data is gone. Cannot reverse-engineer movement patterns.

The engine cannot be weaponized for surveillance because it was never designed to store or transmit continuous tracking data.

## Failure modes

### Engine fails → Trace degrades gracefully

```
Engine: INVALID_WINDOW
  ↓
Trace: "Can't trust this moment. Use last known position."
  ↓
Trace: "Ask user or provide last-seen location as fallback."
```

### Engine is degraded → Trace reduces confidence

```
Engine: DEGRADED_WINDOW with confidence 0.45
  ↓
Trace: "Don't use this for time-critical search."
  ↓
Trace: "Use broader search area or rely on other signals."
```

### Engine is valid → Trace uses fully

```
Engine: VALID_WINDOW with confidence 0.92
  ↓
Trace: "Use this for precise ranking."
  ↓
Trace: "Search this zone first."
```

## Configuration flow

Trace may configure the engine based on device capability:

```rust
let config = if is_low_battery {
    PipelineConfig {
        sample_rate_hz: 25.0,      // Slower sampling
        window_duration_ms: 2000,  // Longer windows
        ..Default::default()
    }
} else {
    PipelineConfig::default()      // Normal 50Hz
};

engine = BatteryOptimizedEngine::new(config);
```

Or dynamically:

```rust
if battery_percent < 20 {
    engine.set_confidence_threshold(0.85);  // Only high-confidence
}

if network_connected {
    // Can afford more processing
    engine.set_confidence_threshold(0.70);
}
```

## Metrics Trace cares about

From `BatteryMetrics`:

```rust
pub struct BatteryMetrics {
    pub samples_processed: u64,        // How much data processed
    pub cpu_time_us: u64,              // CPU burn in microseconds
    pub cpu_percent: f32,              // % of device CPU
    pub power_mw: f32,                 // Estimated milliwatts
    pub elapsed_secs: f64,             // Time window
}

// Derived
pub battery_percent: f32,              // % of 1500mAh daily budget
```

Trace uses this to:

```rust
// Check if we're within budget
if metrics.battery_percent() > 0.8 {
    println!("⚠️ Engine using {:.2}% of daily budget", metrics.battery_percent());
    // Could throttle: longer windows, lower sample rate
}

// Adaptive sampling decision
if metrics.cpu_percent > 80.0 {
    println!("CPU high. Consider switching to SamplingMode::Still");
}
```

## Data retention

The engine outputs motion evidence windows (~1KB each).

Trace's retention strategy is separate:

```
Engine output (every ~1 second):
  → Store in local database
  → Compress/aggregate for long-term storage
  → Use for immediate ranking
  → Expire after N days per privacy policy
```

The engine itself stores nothing permanently.

## Testing integration

Trace can inject synthetic motion windows:

```rust
// Simulate a user stopping
let window = MotionEvidenceWindow {
    context_mode: MotionMode::Still,
    confidence: 0.95,
    transitions: vec![
        TransitionCandidate {
            transition_type: TransitionType::STOP,
            confidence: 0.92,
            timestamp_ms: 1000,
        }
    ],
    ..Default::default()
};

trace_app.handle_motion_window(window);
// Verify item search ranking changed
assert!(trace_app.top_result_changed());
```

## Key principles

1. **The engine extracts facts, Trace interprets them.**
   - Engine: "User stopped at 2:31 PM with 0.85 confidence"
   - Trace: "Likely put down keys in kitchen"

2. **Confidence flows downstream.**
   - High confidence windows → precise ranking
   - Low confidence windows → broad search

3. **Validity gates everything.**
   - Invalid windows are ignored
   - Degraded windows use conservative strategy

4. **Privacy is by design.**
   - Engine never outputs reversible data
   - Trace can share windows, not sensor data

5. **Failures degrade gracefully.**
   - Engine fails → Trace uses fallbacks
   - No data loss, just reduced capability

---

Bottom line: **This engine is the motion physics layer. Trace is the application layer. Together, they solve the interruption problem without surveillance.**
