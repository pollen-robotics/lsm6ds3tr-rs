use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;

use crate::{Error, Lsm6ds3tr};
use embedded_hal::i2c::I2c;

/// Motion-adaptive filter gain.
///
/// The Madgwick filter trusts the accelerometer as a gravity reference. That
/// assumption only holds when the device isn't accelerating, so during fast
/// motion (especially translation) the accelerometer is unreliable.
///
/// While moving, the base `beta` is used. The moment the device becomes still,
/// the gain jumps to `beta_peak` (snapping the estimate back to gravity quickly)
/// and then **decays exponentially back to the base `beta`** with time constant
/// `decay_tau` seconds. This gives a fast correction right after motion stops
/// without the steady-state noise a permanently high gain would cause.
#[derive(Debug, Clone, Copy)]
struct AdaptiveBeta {
    /// Peak gain applied the instant the device becomes still.
    beta_peak: f64,
    /// Time constant (seconds) of the decay from `beta_peak` back to base `beta`.
    decay_tau: f64,
    /// Stillness gate: accel magnitude must be within this many g of 1.0.
    accel_tol_g: f32,
    /// Stillness gate: gyro magnitude must be below this, in rad/s.
    gyro_tol_rad_s: f32,
}

/// Wraps [`Lsm6ds3tr`] with a Madgwick filter to produce orientation quaternions.
pub struct Lsm6ds3trAhrs<I2C> {
    imu: Lsm6ds3tr<I2C>,
    filter: Madgwick<f64>,
    /// Base gain, used while moving (or always, if adaptive is disabled).
    beta: f64,
    adaptive: Option<AdaptiveBeta>,
    /// Seconds the device has been continuously still (drives the gain decay).
    time_still: f64,
}

impl<I2C: I2c> Lsm6ds3trAhrs<I2C> {
    /// Create a new instance.
    ///
    /// * `beta` — filter gain; higher values converge faster but are noisier.
    ///   `0.1` is a reasonable starting point.
    pub fn new(imu: Lsm6ds3tr<I2C>, beta: f64) -> Self {
        Self {
            imu,
            filter: Madgwick::new(1.0 / 104.0, beta),
            beta,
            adaptive: None,
            time_still: 0.0,
        }
    }

    /// Enable motion-adaptive gain.
    ///
    /// While moving, the base `beta` (from [`new`](Self::new)) is used. The moment
    /// the device becomes still — accelerometer magnitude within `accel_tol_g` of
    /// 1 g **and** gyroscope magnitude below `gyro_tol_rad_s` — the gain jumps to
    /// `beta_peak` to snap the estimate back to gravity, then decays exponentially
    /// back to the base `beta` over `decay_tau` seconds. This gives fast
    /// reconvergence after motion *without* the steady-state noise a permanently
    /// high gain would cause.
    ///
    /// Reasonable starting values: `beta_peak = 0.6`, `decay_tau = 0.4`,
    /// `accel_tol_g = 0.15`, `gyro_tol_rad_s = 0.15` (≈ 8.6 °/s), with a base
    /// `beta` around `0.1`.
    pub fn set_motion_adaptive(
        &mut self,
        beta_peak: f64,
        decay_tau: f64,
        accel_tol_g: f32,
        gyro_tol_rad_s: f32,
    ) {
        self.adaptive = Some(AdaptiveBeta {
            beta_peak,
            decay_tau,
            accel_tol_g,
            gyro_tol_rad_s,
        });
    }

    /// Disable motion-adaptive gain; the filter reverts to a constant `beta`.
    pub fn clear_motion_adaptive(&mut self) {
        self.adaptive = None;
    }

    /// Read accelerometer and gyroscope, update the Madgwick filter, and return
    /// both the gyroscope reading and the orientation quaternion in one call.
    ///
    /// * `dt` — time since last call, in seconds.
    ///
    /// Returns `(gyro, quaternion)` where:
    /// - `gyro` is `[gx, gy, gz]` in **rad/s** (after any axis remapping)
    /// - `quaternion` is `[w, x, y, z]` (body→world, scalar-first)
    pub fn update(&mut self, dt: f32) -> Result<([f32; 3], [f32; 4]), Error<I2C::Error>> {
        let (ax, ay, az) = self.imu.read_accelerometer()?;
        let (gx, gy, gz) = self.imu.read_gyroscope()?;

        // Pick the gain: while still, boost to beta_peak then decay back to the
        // base beta; while moving, just use the base beta.
        let beta = match self.adaptive {
            Some(a) => {
                let accel_mag = (ax * ax + ay * ay + az * az).sqrt();
                let gyro_mag = (gx * gx + gy * gy + gz * gz).sqrt();
                let still = (accel_mag - 1.0).abs() < a.accel_tol_g && gyro_mag < a.gyro_tol_rad_s;
                if still {
                    self.time_still += dt as f64;
                    // Decay from beta_peak toward the base beta as stillness persists.
                    self.beta + (a.beta_peak - self.beta) * (-self.time_still / a.decay_tau).exp()
                } else {
                    self.time_still = 0.0;
                    self.beta
                }
            }
            None => self.beta,
        };

        // Rebuild with current dt and gain while preserving quaternion state
        self.filter = Madgwick::new_with_quat(dt as f64, beta, self.filter.quat);

        let q = match self.filter.update_imu(
            &Vector3::new(gx as f64, gy as f64, gz as f64),
            &Vector3::new(ax as f64, ay as f64, az as f64),
        ) {
            Ok(q) => *q,
            Err(_) => self.filter.quat,
        };

        let q = q.into_inner();
        Ok(([gx, gy, gz], [q.w as f32, q.i as f32, q.j as f32, q.k as f32]))
    }

    /// Read accelerometer and gyroscope, then update the Madgwick filter.
    ///
    /// * `dt` — time since last call, in seconds.
    ///
    /// Returns the orientation quaternion as `[w, x, y, z]`.
    pub fn get_quaternion(&mut self, dt: f32) -> Result<[f32; 4], Error<I2C::Error>> {
        let (_, quat) = self.update(dt)?;
        Ok(quat)
    }

    /// Access the underlying driver (e.g. to read temperature).
    pub fn imu(&mut self) -> &mut Lsm6ds3tr<I2C> {
        &mut self.imu
    }

    /// Consume the wrapper and return the underlying I2C bus.
    pub fn release(self) -> I2C {
        self.imu.release()
    }
}
