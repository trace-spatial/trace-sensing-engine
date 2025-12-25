# Integration with Trace Components

How to wire sensing engine into Zone Mapper, CEBE-X, and Confidential Computing.

---

## Zone Mapper Integration

**What Zone Mapper needs from sensing engine:**
- Motion transition events (START/STOP/TURN/HESITATION)
- Zone dwell times
- Kinematic signatures (step count, turn angle)

**Implementation:**

```rust
// zone_mapper/src/engine_listener.rs
use trace_sensing::{ImuSample, BatteryOptimizedEngine, PipelineConfig};

pub struct ZoneMapperListener {
    engine: BatteryOptimizedEngine,
    current_zone: Option<ZoneId>,
    transition_buffer: Vec<TransitionEvent>,
}

impl ZoneMapperListener {
    pub fn new() -> Self {
        Self {
            engine: BatteryOptimizedEngine::new(PipelineConfig::default()),
            current_zone: None,
            transition_buffer: Vec::new(),
        }
    }

    pub fn process_imu_sample(&mut self, accel: [f32; 3], gyro: [f32; 3], timestamp_ms: u64) {
        let sample = ImuSample::new(timestamp_ms, accel, gyro);

        if let Some(window) = self.engine.process_sample(sample) {
            // Extract motion evidence
            self.on_motion_window(&window);
        }
    }

    fn on_motion_window(&mut self, window: &MotionEvidenceWindow) {
        // Log segment
        for segment in &window.segments {
            println!("Zone: {:?} for {:.0}ms", segment.mode, segment.duration_ms);
        }

        // Detect transitions - when to scan environment
        for transition in &window.transitions {
            if matches!(transition.transition_type, 
                TransitionType::STOP | TransitionType::START | TransitionType::TURN) {
                // HIGH CONFIDENCE TRANSITION - scan WiFi/BLE NOW
                self.trigger_environmental_scan(transition.timestamp_ms);
            }
        }

        // Update zone dwell time
        if window.context_mode == MotionMode::Still {
            if let Some(zone_id) = self.current_zone {
                zone_db.add_dwell_time(zone_id, window.duration_ms());
            }
        }
    }

    fn trigger_environmental_scan(&self, timestamp_ms: u64) {
        // Scan WiFi and BLE
        let wifi_scan = scan_wifi_networks();
        let ble_scan = scan_ble_beacons();

        // Create zone signature
        let signature = EnvironmentalSignature {
            timestamp_ms,
            wifi_aps: wifi_scan,
            ble_beacons: ble_scan,
            confidence: 0.95, // High because motion triggered it
        };

        // Record in zone graph
        zone_mapper.on_zone_signature(signature);
    }
}
```

**Call from native module:**

```swift
// iOS: MotionEngine native bridge
class MotionEngineModule: NSObject, RCTBridgeModule {
    var zoneListener: ZoneMapperListener!
    var motionManager: CMMotionManager!

    func startMotionCapture() {
        zoneListener = ZoneMapperListener()
        
        motionManager.startDeviceMotionUpdates(to: .main) { [weak self] motion, _ in
            guard let motion = motion else { return }
            
            let accel = [
                Float(motion.userAcceleration.x),
                Float(motion.userAcceleration.y),
                Float(motion.userAcceleration.z)
            ]
            let gyro = [
                Float(motion.rotationRate.x),
                Float(motion.rotationRate.y),
                Float(motion.rotationRate.z)
            ]
            let timestamp = UInt64(Date().timeIntervalSince1970 * 1000)
            
            self?.zoneListener.process_imu_sample(accel, gyro, timestamp)
        }
    }
}
```

---

## CEBE-X Integration

**What CEBE-X needs from sensing engine:**
- Historical motion episodes
- Feature vectors (pause_count, turn_intensity, dwell_time)
- Confidence scores per episode
- Sensor health flags

**Implementation:**

```rust
// cebe_x_engine/src/motion_features.rs
use trace_sensing::MotionEvidenceWindow;

pub struct MotionFeatures {
    pub pause_count: f32,
    pub pause_avg_duration_ms: f32,
    pub total_motion_time_ms: f32,
    pub turn_intensity: f32,     // Average gyro magnitude
    pub direction_changes: u32,
    pub zone_dwell_time_ms: f32,
    pub confidence: f32,          // From window
    pub sensor_health_score: f32, // From window
}

impl MotionFeatures {
    /// Extract features from motion window for ONNX model
    pub fn from_window(window: &MotionEvidenceWindow) -> Self {
        let mut pause_count = 0;
        let mut pause_duration_total = 0u64;
        let mut turn_intensity_sum = 0.0f32;
        let mut direction_changes = 0u32;
        let mut zone_dwell_time = 0u64;

        // Count stillness periods
        for segment in &window.segments {
            if segment.mode == MotionMode::Still {
                pause_count += 1;
                pause_duration_total += segment.duration_ms;
            }
            zone_dwell_time = segment.duration_ms.max(zone_dwell_time);
        }

        // Count turns (transitions)
        for transition in &window.transitions {
            if matches!(transition.transition_type, TransitionType::TURN | TransitionType::DIRECTION_CHANGE) {
                direction_changes += 1;
                turn_intensity_sum += transition.confidence;
            }
        }

        let sensor_health_score = match window.sensor_health {
            SensorHealth::Nominal => 1.0,
            SensorHealth::Degraded => 0.5,
            SensorHealth::Critical => 0.0,
        };

        Self {
            pause_count: pause_count as f32,
            pause_avg_duration_ms: if pause_count > 0 {
                pause_duration_total as f32 / pause_count as f32
            } else {
                0.0
            },
            total_motion_time_ms: window.duration_ms() as f32,
            turn_intensity: turn_intensity_sum / (direction_changes.max(1) as f32),
            direction_changes,
            zone_dwell_time_ms: zone_dwell_time as f32,
            confidence: window.confidence,
            sensor_health_score,
        }
    }

    /// Convert to ONNX input tensor
    pub fn to_tensor(&self) -> Vec<f32> {
        vec![
            self.pause_count,
            self.pause_avg_duration_ms,
            self.total_motion_time_ms,
            self.turn_intensity,
            self.direction_changes as f32,
            self.zone_dwell_time_ms,
            self.confidence,
            self.sensor_health_score,
        ]
    }
}

pub struct CEBEXEngine {
    model: OnnxSession,
    motion_history: VecDeque<MotionFeatures>,
}

impl CEBEXEngine {
    pub fn predict_object_location(
        &mut self,
        object: &str,
        recent_windows: &[MotionEvidenceWindow],
    ) -> Vec<ZoneCandidate> {
        // Extract features from recent motion
        for window in recent_windows {
            if window.validity_state == ValidityState::Valid {
                let features = MotionFeatures::from_window(window);
                self.motion_history.push_back(features);
            }
        }

        // Aggregate features over recent history
        let aggregated = self.aggregate_features();

        // Run ONNX model
        let input = aggregated.to_tensor();
        let output = self.model.run(vec![input]).unwrap();

        // Parse zone scores
        let mut candidates = Vec::new();
        for (zone_id, score) in output.iter().enumerate() {
            candidates.push(ZoneCandidate {
                zone_id: zone_id as u32,
                probability: *score,
                explanation: format!("Pause count: {:.0}, dwell: {:.0}ms", 
                    aggregated.pause_count, 
                    aggregated.zone_dwell_time_ms
                ),
            });
        }

        candidates.sort_by(|a, b| b.probability.partial_cmp(&a.probability).unwrap());
        candidates.truncate(3);

        candidates
    }

    fn aggregate_features(&self) -> MotionFeatures {
        if self.motion_history.is_empty() {
            return MotionFeatures::default();
        }

        let count = self.motion_history.len() as f32;
        let sum: MotionFeatures = self.motion_history.iter()
            .fold(MotionFeatures::default(), |acc, f| {
                MotionFeatures {
                    pause_count: acc.pause_count + f.pause_count,
                    pause_avg_duration_ms: acc.pause_avg_duration_ms + f.pause_avg_duration_ms,
                    total_motion_time_ms: acc.total_motion_time_ms + f.total_motion_time_ms,
                    turn_intensity: acc.turn_intensity + f.turn_intensity,
                    direction_changes: acc.direction_changes + f.direction_changes,
                    zone_dwell_time_ms: acc.zone_dwell_time_ms + f.zone_dwell_time_ms,
                    confidence: acc.confidence + f.confidence,
                    sensor_health_score: acc.sensor_health_score + f.sensor_health_score,
                }
            });

        MotionFeatures {
            pause_count: sum.pause_count / count,
            pause_avg_duration_ms: sum.pause_avg_duration_ms / count,
            total_motion_time_ms: sum.total_motion_time_ms / count,
            turn_intensity: sum.turn_intensity / count,
            direction_changes: (sum.direction_changes as f32 / count) as u32,
            zone_dwell_time_ms: sum.zone_dwell_time_ms / count,
            confidence: sum.confidence / count,
            sensor_health_score: sum.sensor_health_score / count,
        }
    }
}
```

---

## Confidential Computing Integration

**What secure enclave needs:**
- Encrypted motion trace (vector of features)
- Can decrypt only inside TEE
- Must run matching without exposing raw data

**Implementation:**

```rust
// confidential_computing/src/guest_trace.rs
use trace_sensing::{MotionEvidenceWindow, BatteryOptimizedEngine};

pub struct GuestTrace {
    /// Encrypted motion trace vector
    encrypted_features: Vec<u8>,
    /// Ephemeral public key for this trace
    public_key: [u8; 32],
}

impl GuestTrace {
    /// Guest device: collect motion windows and encrypt
    pub fn from_device(
        windows: &[MotionEvidenceWindow],
        host_public_key: &[u8; 32],
    ) -> Self {
        // Aggregate features
        let features = windows.iter()
            .filter(|w| w.validity_state == ValidityState::Valid)
            .map(|w| MotionFeatures::from_window(w).to_tensor())
            .flatten()
            .collect::<Vec<_>>();

        // Encrypt with host's public key (using ECIES)
        let encrypted = encrypt_ecies(host_public_key, &features);

        Self {
            encrypted_features: encrypted,
            public_key: *host_public_key,
        }
    }
}

// Inside secure enclave (Azure Confidential Computing)
pub struct SecureEnclaveService {
    host_zone_graph: HostZoneGraph,
}

impl SecureEnclaveService {
    /// Run matching inside TEE with decrypted data
    pub fn match_guest_in_enclave(
        &self,
        guest_trace: GuestTrace,
        private_key: &[u8; 32], // Only available inside TEE
    ) -> MatchResult {
        // DECRYPT ONLY INSIDE TEE
        let decrypted_features = decrypt_ecies(private_key, &guest_trace.encrypted_features);

        // Run matching algorithm
        let guest_vec = decrypted_features.as_slice();
        let mut zone_scores = Vec::new();

        for zone in &self.host_zone_graph.zones {
            let similarity = cosine_similarity(guest_vec, &zone.embedding);
            zone_scores.push((zone.id, similarity));
        }

        // Sort by score
        zone_scores.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        // RETURN ONLY TOP RESULT + CONFIDENCE
        let top = zone_scores.into_iter().take(1).collect::<Vec<_>>();

        MatchResult {
            zone_ids: top.iter().map(|(id, _)| *id).collect(),
            confidence: top.first().map(|(_, conf)| *conf).unwrap_or(0.0),
        }

        // WIPE decrypted_features from RAM on return
        // (Rust RAII + volatile overwrite)
    }
}

// Guest receives result
pub struct MatchResult {
    pub zone_ids: Vec<u32>,
    pub confidence: f32,
}

// Guest interprets:
// "You probably left it in zone 5 (Bedroom) with 82% confidence"
```

---

## Data Flow Summary

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  Sensing Engine (this library)                          │
│  ↓ Raw IMU → Motion Evidence Windows                    │
│                                                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌────────────────────────────────────────────────┐   │
│  │ Zone Mapper                                    │   │
│  │ (uses: motion transitions, dwell time)        │   │
│  │ (emits: zone graph)                           │   │
│  └────────────────────────────────────────────────┘   │
│                                                         │
│  ┌────────────────────────────────────────────────┐   │
│  │ CEBE-X Engine                                 │   │
│  │ (uses: motion features, history)              │   │
│  │ (emits: probability rankings)                 │   │
│  └────────────────────────────────────────────────┘   │
│                                                         │
│  ┌────────────────────────────────────────────────┐   │
│  │ Confidential Computing (optional)              │   │
│  │ (uses: encrypted motion trace)                │   │
│  │ (emits: zone name + confidence)               │   │
│  └────────────────────────────────────────────────┘   │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Calling Conventions

**All communication is local callbacks, not HTTP:**

```rust
// Define trait for motion listeners
pub trait MotionListener {
    fn on_motion_window(&mut self, window: &MotionEvidenceWindow);
}

// Implement for each component
impl MotionListener for ZoneMapper { ... }
impl MotionListener for CEBEXEngine { ... }

// In main app
let mut listeners: Vec<Box<dyn MotionListener>> = vec![
    Box::new(zone_mapper),
    Box::new(cebex_engine),
];

// Forward all motion events
for window in motion_windows {
    for listener in &mut listeners {
        listener.on_motion_window(&window);
    }
}
```

---

## Summary

- **Sensing Engine**: Extracts motion evidence locally
- **Zone Mapper**: Uses transitions + dwell to build zone graph locally
- **CEBE-X**: Uses motion features to rank objects locally
- **Confidential Computing**: Uses encrypted trace for optional cross-device search

**All offline. No HTTP. No network. <1% battery.**

This is how Trace stays private, fast, and battery-efficient.
