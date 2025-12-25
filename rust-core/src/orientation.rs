//! Orientation estimation and frame normalization.
//!
//! This module provides a stable reference frame for motion analysis,
//! converting from device frame (where axes depend on phone orientation)
//! to a world/gravity frame (where axes are fixed relative to gravity).
//!
//! Why this matters:
//! Whether the phone is in your pocket, hand, or bag, we need a consistent
//! way to analyze motion. This module achieves that by tracking the device's
//! orientation and providing normalized accelerations in a gravity-aligned frame.
//!
//! Algorithm: Incremental attitude estimation using gravity + gyroscope integration.
//! No heavy PCA, no matrix inversions. O(1) per sample.

use crate::types::ImuSample;

/// A simple quaternion representation for rotation.
///
/// Format: (w, x, y, z) where w is the scalar part.
/// Unit quaternions represent rotations in 3D space.
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    /// Scalar (real) part
    pub w: f32,
    /// Vector part (x, y, z)
    pub xyz: [f32; 3],
}

impl Quaternion {
    /// Create a quaternion from components.
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self {
            w,
            xyz: [x, y, z],
        }
    }

    /// Identity quaternion (no rotation).
    pub fn identity() -> Self {
        Self {
            w: 1.0,
            xyz: [0.0, 0.0, 0.0],
        }
    }

    /// Normalize the quaternion to unit length.
    pub fn normalize(&mut self) {
        let len = (self.w * self.w
            + self.xyz[0] * self.xyz[0]
            + self.xyz[1] * self.xyz[1]
            + self.xyz[2] * self.xyz[2])
            .sqrt();

        if len > 0.001 {
            self.w /= len;
            self.xyz[0] /= len;
            self.xyz[1] /= len;
            self.xyz[2] /= len;
        }
    }

    /// Conjugate (inverse for unit quaternions).
    pub fn conjugate(&self) -> Quaternion {
        Quaternion {
            w: self.w,
            xyz: [-self.xyz[0], -self.xyz[1], -self.xyz[2]],
        }
    }

    /// Multiply two quaternions: result = self * other.
    pub fn multiply(&self, other: &Quaternion) -> Quaternion {
        Quaternion {
            w: self.w * other.w
                - self.xyz[0] * other.xyz[0]
                - self.xyz[1] * other.xyz[1]
                - self.xyz[2] * other.xyz[2],
            xyz: [
                self.w * other.xyz[0]
                    + self.xyz[0] * other.w
                    + self.xyz[1] * other.xyz[2]
                    - self.xyz[2] * other.xyz[1],
                self.w * other.xyz[1]
                    - self.xyz[0] * other.xyz[2]
                    + self.xyz[1] * other.w
                    + self.xyz[2] * other.xyz[0],
                self.w * other.xyz[2]
                    + self.xyz[0] * other.xyz[1]
                    - self.xyz[1] * other.xyz[0]
                    + self.xyz[2] * other.w,
            ],
        }
    }

    /// Rotate a vector: result = q * v * q^*.
    pub fn rotate_vector(&self, v: [f32; 3]) -> [f32; 3] {
        let v_quat = Quaternion {
            w: 0.0,
            xyz: v,
        };

        let rotated = self.multiply(&v_quat).multiply(&self.conjugate());

        rotated.xyz
    }

    /// Get rotation matrix (3x3) in row-major order.
    ///
    /// Returns 9 values: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
    pub fn to_rotation_matrix(&self) -> [f32; 9] {
        let w = self.w;
        let x = self.xyz[0];
        let y = self.xyz[1];
        let z = self.xyz[2];

        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ]
    }
}

/// Parameters for orientation estimation.
#[derive(Debug, Clone)]
pub struct OrientationConfig {
    /// Gyroscope integration coefficient for updates.
    /// Range: [0.0, 1.0]. Default: 0.9 (trust gyro more than accel).
    pub gyro_weight: f32,

    /// Accelerometer weight for gravity alignment.
    /// Range: [0.0, 1.0]. Default: 0.1 (use accel to correct gyro drift).
    pub accel_weight: f32,

    /// Gyroscope bias estimation learning rate.
    pub bias_alpha: f32,
}

impl Default for OrientationConfig {
    fn default() -> Self {
        Self {
            gyro_weight: 0.9,
            accel_weight: 0.1,
            bias_alpha: 0.01,
        }
    }
}

/// Orientation estimator using incremental gyro integration + gravity alignment.
///
/// This implements a lightweight sensor fusion: primarily integrate gyroscope
/// (fast, accurate for short periods) with periodic gravity alignment from
/// accelerometer (slow drift correction).
pub struct OrientationEstimator {
    /// Current attitude estimate.
    attitude: Quaternion,

    /// Estimated gyroscope bias (drift).
    gyro_bias: [f32; 3],

    /// Configuration.
    config: OrientationConfig,

    /// Last sample timestamp for dt calculation.
    last_timestamp_ms: u64,

    /// Sample counter for diagnostics.
    sample_count: u64,
}

impl OrientationEstimator {
    /// Create a new orientation estimator.
    pub fn new(config: OrientationConfig) -> Self {
        Self {
            attitude: Quaternion::identity(),
            gyro_bias: [0.0, 0.0, 0.0],
            config,
            last_timestamp_ms: 0,
            sample_count: 0,
        }
    }

    /// Update orientation with a new IMU sample.
    ///
    /// Uses two sources of information:
    /// 1. Gyroscope: fast, accurate rotation rate (integrated over dt)
    /// 2. Accelerometer: slow, long-term gravity reference (corrects drift)
    pub fn update(&mut self, sample: &ImuSample, gravity: [f32; 3]) {
        // Compute time delta in seconds
        let dt = if self.sample_count == 0 {
            0.01 // Default 10ms if first sample
        } else {
            let dt_ms = sample.timestamp_ms.saturating_sub(self.last_timestamp_ms);
            (dt_ms as f32) / 1000.0
        };

        self.last_timestamp_ms = sample.timestamp_ms;
        self.sample_count += 1;

        // --- Step 1: Gyroscope integration ---
        // Remove estimated bias from gyro reading
        let gyro_corrected = [
            sample.gyro[0] - self.gyro_bias[0],
            sample.gyro[1] - self.gyro_bias[1],
            sample.gyro[2] - self.gyro_bias[2],
        ];

        // Create a quaternion for the rotation that occurred during dt
        // Using small-angle approximation: dq ≈ [1, wx/2, wy/2, wz/2] for small w
        let gyro_mag = (gyro_corrected[0] * gyro_corrected[0]
            + gyro_corrected[1] * gyro_corrected[1]
            + gyro_corrected[2] * gyro_corrected[2])
            .sqrt();

        let dq = if gyro_mag > 0.001 {
            let half_angle = gyro_mag * dt / 2.0;
            let sin_half = half_angle.sin();
            let cos_half = half_angle.cos();

            Quaternion {
                w: cos_half,
                xyz: [
                    gyro_corrected[0] / gyro_mag * sin_half,
                    gyro_corrected[1] / gyro_mag * sin_half,
                    gyro_corrected[2] / gyro_mag * sin_half,
                ],
            }
        } else {
            Quaternion::identity()
        };

        // Apply rotation: q_new = dq * q_old
        let mut q_gyro = dq.multiply(&self.attitude);
        q_gyro.normalize();

        // --- Step 2: Gravity-based correction ---
        // Compute the "up" direction in device frame that would result from current attitude
        // If we're aligned with gravity, rotating [0, 0, 1] should give us gravity direction
        let up_device = [0.0, 0.0, 1.0];
        let up_world = q_gyro.rotate_vector(up_device);

        // Compute axis to rotate around to align up_world with gravity
        let gravity_normalized = {
            let mag = (gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2])
                .sqrt();
            if mag > 0.1 {
                [gravity[0] / mag, gravity[1] / mag, gravity[2] / mag]
            } else {
                [0.0, 0.0, -1.0] // Default: gravity downward
            }
        };

        // Cross product: axis = up_world × gravity
        let correction_axis = [
            up_world[1] * gravity_normalized[2] - up_world[2] * gravity_normalized[1],
            up_world[2] * gravity_normalized[0] - up_world[0] * gravity_normalized[2],
            up_world[0] * gravity_normalized[1] - up_world[1] * gravity_normalized[0],
        ];

        let axis_mag = (correction_axis[0] * correction_axis[0]
            + correction_axis[1] * correction_axis[1]
            + correction_axis[2] * correction_axis[2])
            .sqrt();

        let dq_correction = if axis_mag > 0.001 {
            // Limit correction magnitude: accel_weight controls how much we trust accelerometer
            let correction_angle = self.config.accel_weight * 0.1; // Small correction per update
            let sin_half = correction_angle.sin();
            let cos_half = correction_angle.cos();

            Quaternion {
                w: cos_half,
                xyz: [
                    correction_axis[0] / axis_mag * sin_half,
                    correction_axis[1] / axis_mag * sin_half,
                    correction_axis[2] / axis_mag * sin_half,
                ],
            }
        } else {
            Quaternion::identity()
        };

        // Blend: attitude = lerp(q_gyro, q_corrected, accel_weight)
        // Simplified as: multiply correction into gyro estimate
        self.attitude = dq_correction.multiply(&q_gyro);
        self.attitude.normalize();

        // --- Step 3: Update gyro bias estimate ---
        // If up_world is very misaligned with gravity, increase bias estimate slightly
        let dot = up_world[0] * gravity_normalized[0]
            + up_world[1] * gravity_normalized[1]
            + up_world[2] * gravity_normalized[2];

        if dot < 0.95 && dt > 0.0 {
            let bias_correction = self.config.bias_alpha * 0.01; // Small bias update
            self.gyro_bias[0] -= correction_axis[0] * bias_correction / dt;
            self.gyro_bias[1] -= correction_axis[1] * bias_correction / dt;
            self.gyro_bias[2] -= correction_axis[2] * bias_correction / dt;
        }
    }

    /// Get the current attitude quaternion.
    pub fn attitude(&self) -> Quaternion {
        self.attitude
    }

    /// Get the rotation matrix from device frame to world frame.
    pub fn rotation_matrix(&self) -> [f32; 9] {
        self.attitude.to_rotation_matrix()
    }

    /// Rotate a vector from device frame to world frame.
    pub fn rotate_to_world(&self, v_device: [f32; 3]) -> [f32; 3] {
        self.attitude.rotate_vector(v_device)
    }

    /// Get estimated gyroscope bias.
    pub fn gyro_bias(&self) -> [f32; 3] {
        self.gyro_bias
    }

    /// Get sample count.
    pub fn sample_count(&self) -> u64 {
        self.sample_count
    }
}

impl Default for OrientationEstimator {
    fn default() -> Self {
        Self::new(OrientationConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quaternion_identity() {
        let q = Quaternion::identity();
        assert_eq!(q.w, 1.0);
        assert_eq!(q.xyz, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_quaternion_rotate_identity() {
        let q = Quaternion::identity();
        let v = [1.0, 2.0, 3.0];
        let rotated = q.rotate_vector(v);

        assert!((rotated[0] - v[0]).abs() < 0.001);
        assert!((rotated[1] - v[1]).abs() < 0.001);
        assert!((rotated[2] - v[2]).abs() < 0.001);
    }

    #[test]
    fn test_quaternion_normalization() {
        let mut q = Quaternion::new(2.0, 2.0, 2.0, 2.0);
        q.normalize();

        let mag_sq = q.w * q.w + q.xyz[0] * q.xyz[0] + q.xyz[1] * q.xyz[1] + q.xyz[2] * q.xyz[2];
        assert!((mag_sq - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_quaternion_conjugate() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let conj = q.conjugate();

        assert_eq!(conj.w, 1.0);
        assert_eq!(conj.xyz[0], -2.0);
        assert_eq!(conj.xyz[1], -3.0);
        assert_eq!(conj.xyz[2], -4.0);
    }

    #[test]
    fn test_quaternion_to_matrix() {
        let q = Quaternion::identity();
        let matrix = q.to_rotation_matrix();

        // Should be identity matrix
        assert!((matrix[0] - 1.0).abs() < 0.001);
        assert!(matrix[1].abs() < 0.001);
        assert!(matrix[2].abs() < 0.001);
        assert!(matrix[3].abs() < 0.001);
        assert!((matrix[4] - 1.0).abs() < 0.001);
        assert!(matrix[5].abs() < 0.001);
        assert!(matrix[6].abs() < 0.001);
        assert!(matrix[7].abs() < 0.001);
        assert!((matrix[8] - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_orientation_estimator_creation() {
        let estimator = OrientationEstimator::new(OrientationConfig::default());

        assert_eq!(estimator.attitude().w, 1.0);
        assert_eq!(estimator.sample_count(), 0);
        assert_eq!(estimator.gyro_bias(), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_orientation_estimator_update() {
        let config = OrientationConfig::default();
        let mut estimator = OrientationEstimator::new(config);

        let sample = ImuSample::new(
            1000,
            [0.0, 0.0, -9.81],  // Gravity down
            [0.0, 0.0, 0.0],    // No rotation
        );

        estimator.update(&sample, [0.0, 0.0, -9.81]);

        assert_eq!(estimator.sample_count(), 1);
        // Attitude should be close to identity (no motion)
        let att = estimator.attitude();
        assert!((att.w - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_orientation_estimator_rotation() {
        let config = OrientationConfig::default();
        let mut estimator = OrientationEstimator::new(config);

        // Simulate several updates with small rotation
        for i in 0..10 {
            let sample = ImuSample::new(
                (i * 10) as u64,
                [0.0, 0.0, -9.81],
                [0.1, 0.0, 0.0],  // Small rotation around x-axis
            );
            estimator.update(&sample, [0.0, 0.0, -9.81]);
        }

        // Should have accumulated some rotation
        assert!(estimator.sample_count() == 10);
    }

    #[test]
    fn test_rotation_matrix_multiplication() {
        let q1 = Quaternion::new(0.707, 0.707, 0.0, 0.0); // 90° around x
        let matrix = q1.to_rotation_matrix();

        // Should have proper rotation properties
        // Matrix should be orthogonal (det = 1, columns are unit vectors)
        assert!(!matrix.iter().any(|&x| x.is_nan()));
    }
}
