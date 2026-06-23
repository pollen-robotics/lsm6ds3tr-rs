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

/// Transport-agnostic Madgwick AHRS.
///
/// Holds only the filter state — it does **not** own a sensor or a bus. Feed it
/// samples from any source with [`update`](Self::update): the on-chip I2C driver
/// ([`Lsm6ds3trAhrs`]) uses it, and so can a caller relaying the same raw
/// registers over another transport (e.g. a Dynamixel bus). Identical math
/// either way.
pub struct MadgwickAhrs {
    filter: Madgwick<f64>,
    /// Base gain, used while moving (or always, if adaptive is disabled).
    beta: f64,
    adaptive: Option<AdaptiveBeta>,
    /// Seconds the device has been continuously still (drives the gain decay).
    time_still: f64,
}

impl MadgwickAhrs {
    /// Create a new filter.
    ///
    /// * `beta` — filter gain; higher values converge faster but are noisier.
    ///   `0.1` is a reasonable starting point.
    pub fn new(beta: f64) -> Self {
        Self {
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
    /// back to the base `beta` over `decay_tau` seconds.
    ///
    /// Reasonable starting values: `beta_peak = 0.6`, `decay_tau = 0.4`,
    /// `accel_tol_g = 0.15`, `gyro_tol_rad_s = 0.15`, base `beta` around `0.1`.
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

    /// Update the filter with one sample and return `(gyro, quaternion)`.
    ///
    /// * `gyro` — `[gx, gy, gz]` in **rad/s** (already axis-remapped).
    /// * `accel` — `[ax, ay, az]` in **g** (already axis-remapped). Units must be
    ///   g: the motion-adaptive stillness gate compares the magnitude to 1.0.
    /// * `dt` — time since last call, in seconds.
    ///
    /// Returns the gyro unchanged plus the orientation quaternion `[w, x, y, z]`
    /// (body→world, scalar-first).
    pub fn update(&mut self, gyro: [f32; 3], accel: [f32; 3], dt: f32) -> ([f32; 3], [f32; 4]) {
        let [gx, gy, gz] = gyro;
        let [ax, ay, az] = accel;

        // Pick the gain: while still, boost to beta_peak then decay back to the
        // base beta; while moving, just use the base beta.
        let beta = match self.adaptive {
            Some(a) => {
                let accel_mag = (ax * ax + ay * ay + az * az).sqrt();
                let gyro_mag = (gx * gx + gy * gy + gz * gz).sqrt();
                let still = (accel_mag - 1.0).abs() < a.accel_tol_g && gyro_mag < a.gyro_tol_rad_s;
                if still {
                    self.time_still += dt as f64;
                    self.beta + (a.beta_peak - self.beta) * (-self.time_still / a.decay_tau).exp()
                } else {
                    self.time_still = 0.0;
                    self.beta
                }
            }
            None => self.beta,
        };

        // Rebuild with current dt and gain while preserving quaternion state.
        self.filter = Madgwick::new_with_quat(dt as f64, beta, self.filter.quat);

        let q = match self.filter.update_imu(
            &Vector3::new(gx as f64, gy as f64, gz as f64),
            &Vector3::new(ax as f64, ay as f64, az as f64),
        ) {
            Ok(q) => *q,
            Err(_) => self.filter.quat,
        };

        let q = q.into_inner();
        ([gx, gy, gz], [q.w as f32, q.i as f32, q.j as f32, q.k as f32])
    }

    /// Current orientation quaternion `[w, x, y, z]` without updating.
    pub fn quat(&self) -> [f32; 4] {
        let q = self.filter.quat.into_inner();
        [q.w as f32, q.i as f32, q.j as f32, q.k as f32]
    }
}

/// Wraps [`Lsm6ds3tr`] (I2C) with a [`MadgwickAhrs`] to produce orientation
/// quaternions directly from the sensor.
pub struct Lsm6ds3trAhrs<I2C> {
    imu: Lsm6ds3tr<I2C>,
    ahrs: MadgwickAhrs,
}

impl<I2C: I2c> Lsm6ds3trAhrs<I2C> {
    /// Create a new instance. `beta` — filter gain (see [`MadgwickAhrs::new`]).
    pub fn new(imu: Lsm6ds3tr<I2C>, beta: f64) -> Self {
        Self {
            imu,
            ahrs: MadgwickAhrs::new(beta),
        }
    }

    /// Enable motion-adaptive gain (see [`MadgwickAhrs::set_motion_adaptive`]).
    pub fn set_motion_adaptive(
        &mut self,
        beta_peak: f64,
        decay_tau: f64,
        accel_tol_g: f32,
        gyro_tol_rad_s: f32,
    ) {
        self.ahrs
            .set_motion_adaptive(beta_peak, decay_tau, accel_tol_g, gyro_tol_rad_s);
    }

    /// Disable motion-adaptive gain.
    pub fn clear_motion_adaptive(&mut self) {
        self.ahrs.clear_motion_adaptive();
    }

    /// Read accelerometer and gyroscope, update the filter, and return both the
    /// gyroscope reading and the orientation quaternion.
    ///
    /// * `dt` — time since last call, in seconds.
    ///
    /// Returns `(gyro, quaternion)`: `gyro` is `[gx, gy, gz]` in rad/s (after
    /// remap); `quaternion` is `[w, x, y, z]` (body→world, scalar-first).
    pub fn update(&mut self, dt: f32) -> Result<([f32; 3], [f32; 4]), Error<I2C::Error>> {
        let (ax, ay, az) = self.imu.read_accelerometer()?;
        let (gx, gy, gz) = self.imu.read_gyroscope()?;
        Ok(self.ahrs.update([gx, gy, gz], [ax, ay, az], dt))
    }

    /// Read sensors, update the filter, and return only the quaternion.
    pub fn get_quaternion(&mut self, dt: f32) -> Result<[f32; 4], Error<I2C::Error>> {
        let (_, quat) = self.update(dt)?;
        Ok(quat)
    }

    /// Access the underlying driver (e.g. to read temperature).
    pub fn imu(&mut self) -> &mut Lsm6ds3tr<I2C> {
        &mut self.imu
    }

    /// Access the inner transport-agnostic filter.
    pub fn ahrs(&mut self) -> &mut MadgwickAhrs {
        &mut self.ahrs
    }

    /// Consume the wrapper and return the underlying I2C bus.
    pub fn release(self) -> I2C {
        self.imu.release()
    }
}
