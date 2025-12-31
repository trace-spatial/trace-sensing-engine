//! Orientation Estimation using Madgwick AHRS Filter.
//!
//! This module provides stable attitude (orientation) estimation by fusing
//! accelerometer, gyroscope, and optionally magnetometer data using the
//! Madgwick Attitude and Heading Reference System (AHRS) algorithm.
//!
//! The Madgwick filter uses gradient descent optimization to find the
//! quaternion orientation that best aligns the measured gravity (and
//! optionally magnetic field) with the expected reference directions.
//!
//! Benefits over simple complementary filters:
//! - Better accuracy during dynamic motion
//! - Automatic gyro bias compensation
//! - Tunable convergence rate (beta parameter)
//! - Works with any phone orientation
//!
//! Reference: Madgwick, S. O. H. (2010). "An efficient orientation filter
//! for inertial and inertial/magnetic sensor arrays."

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
    /// Madgwick filter gain (beta parameter).
    /// Higher values = faster convergence but more noise sensitivity.
    /// Typical range: 0.01 - 0.5. Default: 0.1 for moderate motion.
    pub beta: f32,
    
    /// Gyroscope measurement error (rad/s).
    /// Used for adaptive beta calculation.
    pub gyro_error: f32,
    
    /// Gyroscope drift rate (rad/s/s).
    /// Used for gyro bias estimation.
    pub gyro_drift: f32,
    
    /// Whether to use magnetometer for heading correction.
    pub use_magnetometer: bool,
    
    /// Magnetic field reference (local magnetic north).
    /// Only used if use_magnetometer is true.
    pub magnetic_reference: [f32; 3],
    
    /// Zeta parameter for gyro bias drift compensation.
    pub zeta: f32,
}

impl Default for OrientationConfig {
    fn default() -> Self {
        Self {
            beta: 0.1,
            gyro_error: 0.05,   // ~3 deg/s
            gyro_drift: 0.001,  // ~0.06 deg/s/s
            use_magnetometer: false,
            magnetic_reference: [1.0, 0.0, 0.0], // Magnetic north in XY plane
            zeta: 0.015,
        }
    }
}

/// Madgwick AHRS (Attitude and Heading Reference System) Filter.
///
/// This implements the Madgwick gradient descent algorithm for sensor fusion.
/// It provides accurate orientation estimation from IMU data and is robust
/// to magnetic disturbances and varying motion dynamics.
pub struct MadgwickAHRS {
    /// Current attitude quaternion (w, x, y, z).
    quaternion: Quaternion,
    
    /// Configuration parameters.
    config: OrientationConfig,
    
    /// Estimated gyroscope bias.
    gyro_bias: [f32; 3],
    
    /// Previous timestamp for dt calculation.
    prev_timestamp_ms: u64,
    
    /// Sample counter.
    sample_count: u64,
    
    /// Adaptive beta (adjusted based on motion).
    adaptive_beta: f32,
}

impl MadgwickAHRS {
    /// Create a new Madgwick AHRS filter.
    pub fn new(config: OrientationConfig) -> Self {
        Self {
            quaternion: Quaternion::identity(),
            config: config.clone(),
            gyro_bias: [0.0; 3],
            prev_timestamp_ms: 0,
            sample_count: 0,
            adaptive_beta: config.beta,
        }
    }
    
    /// Create with default configuration.
    pub fn default_filter() -> Self {
        Self::new(OrientationConfig::default())
    }
    
    /// Update orientation with a new IMU sample (accelerometer + gyroscope only).
    pub fn update(&mut self, sample: &ImuSample) {
        // Calculate dt
        let dt = self.calculate_dt(sample.timestamp_ms);
        self.prev_timestamp_ms = sample.timestamp_ms;
        self.sample_count += 1;
        
        // Get sensor readings
        let ax = sample.accel[0];
        let ay = sample.accel[1];
        let az = sample.accel[2];
        
        // Apply gyro bias correction
        let gx = sample.gyro[0] - self.gyro_bias[0];
        let gy = sample.gyro[1] - self.gyro_bias[1];
        let gz = sample.gyro[2] - self.gyro_bias[2];
        
        // Current quaternion
        let q = &self.quaternion;
        let q0 = q.w;
        let q1 = q.xyz[0];
        let q2 = q.xyz[1];
        let q3 = q.xyz[2];
        
        // Rate of change of quaternion from gyroscope
        let q_dot_omega = Quaternion::new(
            0.5 * (-q1 * gx - q2 * gy - q3 * gz),
            0.5 * (q0 * gx + q2 * gz - q3 * gy),
            0.5 * (q0 * gy - q1 * gz + q3 * gx),
            0.5 * (q0 * gz + q1 * gy - q2 * gx),
        );
        
        // Normalize accelerometer measurement
        let accel_norm = (ax * ax + ay * ay + az * az).sqrt();
        
        if accel_norm > 0.1 {
            let ax = ax / accel_norm;
            let ay = ay / accel_norm;
            let az = az / accel_norm;
            
            // Auxiliary variables for readability
            let _2q0 = 2.0 * q0;
            let _2q1 = 2.0 * q1;
            let _2q2 = 2.0 * q2;
            let _2q3 = 2.0 * q3;
            let _4q0 = 4.0 * q0;
            let _4q1 = 4.0 * q1;
            let _4q2 = 4.0 * q2;
            let _8q1 = 8.0 * q1;
            let _8q2 = 8.0 * q2;
            let q0q0 = q0 * q0;
            let q1q1 = q1 * q1;
            let q2q2 = q2 * q2;
            let q3q3 = q3 * q3;
            
            // Gradient descent algorithm corrective step
            let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            let s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            let s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            let s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
            
            // Normalize step magnitude
            let step_norm = (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3).sqrt();
            let (s0, s1, s2, s3) = if step_norm > 0.0 {
                (s0 / step_norm, s1 / step_norm, s2 / step_norm, s3 / step_norm)
            } else {
                (0.0, 0.0, 0.0, 0.0)
            };
            
            // Compute gyro bias estimate
            let gyro_bias_error = [
                _2q0 * s1 - _2q1 * s0 - _2q2 * s3 + _2q3 * s2,
                _2q0 * s2 + _2q1 * s3 - _2q2 * s0 - _2q3 * s1,
                _2q0 * s3 - _2q1 * s2 + _2q2 * s1 - _2q3 * s0,
            ];
            
            // Update gyro bias with zeta
            self.gyro_bias[0] += gyro_bias_error[0] * dt * self.config.zeta;
            self.gyro_bias[1] += gyro_bias_error[1] * dt * self.config.zeta;
            self.gyro_bias[2] += gyro_bias_error[2] * dt * self.config.zeta;
            
            // Apply feedback step
            let beta = self.adaptive_beta;
            self.quaternion = Quaternion::new(
                q0 + (q_dot_omega.w - beta * s0) * dt,
                q1 + (q_dot_omega.xyz[0] - beta * s1) * dt,
                q2 + (q_dot_omega.xyz[1] - beta * s2) * dt,
                q3 + (q_dot_omega.xyz[2] - beta * s3) * dt,
            );
        } else {
            // Pure gyro integration when accelerometer is unreliable
            self.quaternion = Quaternion::new(
                q0 + q_dot_omega.w * dt,
                q1 + q_dot_omega.xyz[0] * dt,
                q2 + q_dot_omega.xyz[1] * dt,
                q3 + q_dot_omega.xyz[2] * dt,
            );
        }
        
        // Normalize quaternion
        self.quaternion.normalize();
        
        // Update adaptive beta based on acceleration dynamics
        self.update_adaptive_beta(accel_norm);
    }
    
    /// Update orientation with magnetometer data (full 9-DOF fusion).
    pub fn update_with_mag(&mut self, sample: &ImuSample) {
        let mag = match sample.mag {
            Some(m) => m,
            None => {
                // Fall back to 6-DOF update
                self.update(sample);
                return;
            }
        };
        
        // Calculate dt
        let dt = self.calculate_dt(sample.timestamp_ms);
        self.prev_timestamp_ms = sample.timestamp_ms;
        self.sample_count += 1;
        
        // Get sensor readings
        let ax = sample.accel[0];
        let ay = sample.accel[1];
        let az = sample.accel[2];
        let mx = mag[0];
        let my = mag[1];
        let mz = mag[2];
        
        // Apply gyro bias correction
        let gx = sample.gyro[0] - self.gyro_bias[0];
        let gy = sample.gyro[1] - self.gyro_bias[1];
        let gz = sample.gyro[2] - self.gyro_bias[2];
        
        let q = &self.quaternion;
        let q0 = q.w;
        let q1 = q.xyz[0];
        let q2 = q.xyz[1];
        let q3 = q.xyz[2];
        
        // Rate of change from gyro
        let q_dot_omega = Quaternion::new(
            0.5 * (-q1 * gx - q2 * gy - q3 * gz),
            0.5 * (q0 * gx + q2 * gz - q3 * gy),
            0.5 * (q0 * gy - q1 * gz + q3 * gx),
            0.5 * (q0 * gz + q1 * gy - q2 * gx),
        );
        
        // Normalize accelerometer
        let accel_norm = (ax * ax + ay * ay + az * az).sqrt();
        
        // Normalize magnetometer
        let mag_norm = (mx * mx + my * my + mz * mz).sqrt();
        
        if accel_norm > 0.1 && mag_norm > 0.1 {
            let ax = ax / accel_norm;
            let ay = ay / accel_norm;
            let az = az / accel_norm;
            let mx = mx / mag_norm;
            let my = my / mag_norm;
            let mz = mz / mag_norm;
            
            // Reference direction of Earth's magnetic field
            let hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + 2.0*my*(q1*q2 - q0*q3) + 2.0*mz*(q1*q3 + q0*q2);
            let hy = 2.0*mx*(q0*q3 + q1*q2) + my*(q0*q0 - q1*q1 + q2*q2 - q3*q3) + 2.0*mz*(q2*q3 - q0*q1);
            let bx = (hx * hx + hy * hy).sqrt();
            let bz = 2.0*mx*(q1*q3 - q0*q2) + 2.0*my*(q0*q1 + q2*q3) + mz*(q0*q0 - q1*q1 - q2*q2 + q3*q3);
            
            // Gradient descent step (combined accel + mag)
            let _2q0mx = 2.0 * q0 * mx;
            let _2q0my = 2.0 * q0 * my;
            let _2q0mz = 2.0 * q0 * mz;
            let _2q1mx = 2.0 * q1 * mx;
            let _2q0 = 2.0 * q0;
            let _2q1 = 2.0 * q1;
            let _2q2 = 2.0 * q2;
            let _2q3 = 2.0 * q3;
            let _2bx = 2.0 * bx;
            let _2bz = 2.0 * bz;
            let _4bx = 4.0 * bx;
            let _4bz = 4.0 * bz;
            let q0q0 = q0 * q0;
            let q0q1 = q0 * q1;
            let q0q2 = q0 * q2;
            let q0q3 = q0 * q3;
            let q1q1 = q1 * q1;
            let q1q2 = q1 * q2;
            let q1q3 = q1 * q3;
            let q2q2 = q2 * q2;
            let q2q3 = q2 * q3;
            let q3q3 = q3 * q3;
            
            // Objective function
            let f1 = 2.0*(q1q3 - q0q2) - ax;
            let f2 = 2.0*(q0q1 + q2q3) - ay;
            let f3 = 2.0*(0.5 - q1q1 - q2q2) - az;
            let f4 = _2bx*(0.5 - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx;
            let f5 = _2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my;
            let f6 = _2bx*(q0q2 + q1q3) + _2bz*(0.5 - q1q1 - q2q2) - mz;
            
            // Jacobian
            let j11_14 = _2q2;
            let j12_13 = _2q3;
            let j21_24 = _2q1;
            let j22_23 = _2q0;
            let j31 = 0.0;
            let j32 = -4.0*q1;
            let j33 = -4.0*q2;
            let j41 = -_2bz*q2;
            let j42 = _2bz*q3;
            let j43 = -_4bx*q2 - _2bz*q0;
            let j44 = -_4bx*q3 + _2bz*q1;
            let j51 = -_2bx*q3 + _2bz*q1;
            let j52 = _2bx*q2 + _2bz*q0;
            let j53 = _2bx*q1 + _2bz*q3;
            let j54 = -_2bx*q0 + _2bz*q2;
            let j61 = _2bx*q2;
            let j62 = _2bx*q3 - _4bz*q1;
            let j63 = _2bx*q0 - _4bz*q2;
            let j64 = _2bx*q1;
            
            // Gradient
            let s0 = -j12_13*f1 + j22_23*f2 + j41*f4 + j51*f5 + j61*f6;
            let s1 = j11_14*f1 + j21_24*f2 + j31*f3 + j42*f4 + j52*f5 + j62*f6;
            let s2 = -j11_14*f1 + j21_24*f2 + j32*f3 + j43*f4 + j53*f5 + j63*f6;
            let s3 = j12_13*f1 + j22_23*f2 + j33*f3 + j44*f4 + j54*f5 + j64*f6;
            
            // Normalize
            let step_norm = (s0*s0 + s1*s1 + s2*s2 + s3*s3).sqrt();
            let (s0, s1, s2, s3) = if step_norm > 0.0 {
                (s0/step_norm, s1/step_norm, s2/step_norm, s3/step_norm)
            } else {
                (0.0, 0.0, 0.0, 0.0)
            };
            
            // Apply feedback
            let beta = self.adaptive_beta;
            self.quaternion = Quaternion::new(
                q0 + (q_dot_omega.w - beta * s0) * dt,
                q1 + (q_dot_omega.xyz[0] - beta * s1) * dt,
                q2 + (q_dot_omega.xyz[1] - beta * s2) * dt,
                q3 + (q_dot_omega.xyz[2] - beta * s3) * dt,
            );
        } else {
            // Pure gyro
            self.quaternion = Quaternion::new(
                q0 + q_dot_omega.w * dt,
                q1 + q_dot_omega.xyz[0] * dt,
                q2 + q_dot_omega.xyz[1] * dt,
                q3 + q_dot_omega.xyz[2] * dt,
            );
        }
        
        self.quaternion.normalize();
    }
    
    /// Get the current attitude quaternion.
    pub fn quaternion(&self) -> Quaternion {
        self.quaternion
    }
    
    /// Get the rotation matrix from device to world frame.
    pub fn rotation_matrix(&self) -> [f32; 9] {
        self.quaternion.to_rotation_matrix()
    }
    
    /// Rotate a vector from device frame to world frame.
    pub fn rotate_to_world(&self, v_device: [f32; 3]) -> [f32; 3] {
        self.quaternion.rotate_vector(v_device)
    }
    
    /// Get Euler angles (roll, pitch, yaw) in radians.
    pub fn euler_angles(&self) -> [f32; 3] {
        let q = &self.quaternion;
        let q0 = q.w;
        let q1 = q.xyz[0];
        let q2 = q.xyz[1];
        let q3 = q.xyz[2];
        
        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
        let cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
        let roll = sinr_cosp.atan2(cosr_cosp);
        
        // Pitch (y-axis rotation)
        let sinp = 2.0 * (q0 * q2 - q3 * q1);
        let pitch = if sinp.abs() >= 1.0 {
            std::f32::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };
        
        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
        let cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
        let yaw = siny_cosp.atan2(cosy_cosp);
        
        [roll, pitch, yaw]
    }
    
    /// Get gravity vector in device frame.
    pub fn gravity_device_frame(&self) -> [f32; 3] {
        // Gravity is [0, 0, -1] in world frame
        // Rotate to device frame using conjugate
        let world_gravity = [0.0, 0.0, -9.81];
        let q_conj = self.quaternion.conjugate();
        q_conj.rotate_vector(world_gravity)
    }
    
    /// Get estimated gyroscope bias.
    pub fn gyro_bias(&self) -> [f32; 3] {
        self.gyro_bias
    }
    
    /// Reset to identity orientation.
    pub fn reset(&mut self) {
        self.quaternion = Quaternion::identity();
        self.gyro_bias = [0.0; 3];
        self.prev_timestamp_ms = 0;
        self.sample_count = 0;
        self.adaptive_beta = self.config.beta;
    }
    
    /// Get sample count.
    pub fn sample_count(&self) -> u64 {
        self.sample_count
    }
    
    fn calculate_dt(&self, timestamp_ms: u64) -> f32 {
        if self.prev_timestamp_ms == 0 {
            0.02 // Default 50Hz
        } else {
            let dt_ms = timestamp_ms.saturating_sub(self.prev_timestamp_ms);
            (dt_ms as f32 / 1000.0).clamp(0.001, 0.1)
        }
    }
    
    fn update_adaptive_beta(&mut self, accel_norm: f32) {
        // Reduce beta during high dynamics (far from 1g)
        let deviation = (accel_norm - 9.81).abs();
        
        if deviation > 3.0 {
            // High dynamics - trust gyro more
            self.adaptive_beta = self.config.beta * 0.5;
        } else if deviation > 1.0 {
            // Moderate dynamics
            self.adaptive_beta = self.config.beta * 0.8;
        } else {
            // Near 1g - can trust accelerometer more
            self.adaptive_beta = self.config.beta;
        }
    }
}

impl Default for MadgwickAHRS {
    fn default() -> Self {
        Self::default_filter()
    }
}

// Backward compatibility alias
pub type OrientationEstimator = MadgwickAHRS;

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
    fn test_madgwick_creation() {
        let filter = MadgwickAHRS::default_filter();
        assert_eq!(filter.quaternion().w, 1.0);
        assert_eq!(filter.sample_count(), 0);
    }

    #[test]
    fn test_madgwick_update_stationary() {
        let mut filter = MadgwickAHRS::default_filter();

        // Stationary: gravity pointing down
        for i in 0..50 {
            let sample = ImuSample::new(
                i * 20,
                [0.0, 0.0, -9.81],
                [0.0, 0.0, 0.0],
            );
            filter.update(&sample);
        }

        // Should converge to identity-ish
        let q = filter.quaternion();
        assert!(q.w.abs() > 0.9, "Should converge, got w={}", q.w);
    }

    #[test]
    fn test_madgwick_update_rotation() {
        let mut filter = MadgwickAHRS::default_filter();

        // Simulate rotation around z-axis
        for i in 0..100 {
            let sample = ImuSample::new(
                i * 20,
                [0.0, 0.0, -9.81],
                [0.0, 0.0, 0.5], // Rotating around z
            );
            filter.update(&sample);
        }

        // Should have accumulated rotation
        let euler = filter.euler_angles();
        assert!(euler[2].abs() > 0.1, "Should have yaw rotation");
    }

    #[test]
    fn test_euler_angles() {
        let filter = MadgwickAHRS::default_filter();
        let euler = filter.euler_angles();
        
        // Identity should have zero euler angles
        assert!(euler[0].abs() < 0.01);
        assert!(euler[1].abs() < 0.01);
        assert!(euler[2].abs() < 0.01);
    }

    #[test]
    fn test_gravity_device_frame() {
        let filter = MadgwickAHRS::default_filter();
        let gravity = filter.gravity_device_frame();
        
        // At identity, gravity should be [0, 0, -9.81]
        assert!(gravity[0].abs() < 0.1);
        assert!(gravity[1].abs() < 0.1);
        assert!((gravity[2] + 9.81).abs() < 0.1);
    }

    #[test]
    fn test_reset() {
        let mut filter = MadgwickAHRS::default_filter();
        
        // Process some samples
        for i in 0..20 {
            let sample = ImuSample::new(i * 20, [0.0, 0.0, -9.81], [0.1, 0.0, 0.0]);
            filter.update(&sample);
        }
        
        filter.reset();
        
        assert_eq!(filter.quaternion().w, 1.0);
        assert_eq!(filter.sample_count(), 0);
        assert_eq!(filter.gyro_bias(), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_rotation_matrix_multiplication() {
        let q1 = Quaternion::new(0.707, 0.707, 0.0, 0.0); // 90° around x
        let matrix = q1.to_rotation_matrix();

        // Should have proper rotation properties
        assert!(!matrix.iter().any(|&x| x.is_nan()));
    }

    #[test]
    fn test_adaptive_beta() {
        let mut filter = MadgwickAHRS::default_filter();
        
        // High dynamics should reduce beta
        let sample = ImuSample::new(0, [5.0, 5.0, 5.0], [0.0, 0.0, 0.0]);
        filter.update(&sample);
        
        // Beta should be reduced from default
        assert!(filter.adaptive_beta < filter.config.beta);
    }
}
