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
/// While moving, the base `beta` is used. Once the device has been continuously
/// still for `still_min_s` seconds, the gain jumps to `beta_peak` (snapping the
/// estimate back to gravity quickly) and then **decays exponentially back to
/// the base `beta`** with time constant `decay_tau` seconds. This gives a fast
/// correction after motion stops without the steady-state noise a permanently
/// high gain would cause.
///
/// `still_min_s` exists because rhythmic motion (e.g. a walking robot's double-
/// support phase) contains brief instants that pass the stillness gate; without
/// a minimum still time each of those instants fires a `beta_peak` burst that
/// yanks the estimate toward the motion-corrupted accelerometer — worse than a
/// constant gain. Set it longer than any quiet gap inside the motion (0.3–0.5 s
/// for walking); genuine standing exceeds it almost immediately.
#[derive(Debug, Clone, Copy)]
struct AdaptiveBeta {
    /// Peak gain applied once the device has been still for `still_min_s`.
    beta_peak: f64,
    /// Time constant (seconds) of the decay from `beta_peak` back to base `beta`.
    decay_tau: f64,
    /// Stillness gate: accel magnitude must be within this many g of 1.0.
    accel_tol_g: f32,
    /// Stillness gate: gyro magnitude must be below this, in rad/s.
    gyro_tol_rad_s: f32,
    /// Continuous stillness required before the gain boost engages (seconds).
    still_min_s: f64,
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
    /// While moving, the base `beta` (from [`new`](Self::new)) is used. Once the
    /// device has been continuously still — accelerometer magnitude within
    /// `accel_tol_g` of 1 g **and** gyroscope magnitude below `gyro_tol_rad_s` —
    /// for `still_min_s` seconds, the gain jumps to `beta_peak` to snap the
    /// estimate back to gravity, then decays exponentially back to the base
    /// `beta` over `decay_tau` seconds.
    ///
    /// `still_min_s` guards against rhythmic motion (walking gait) whose brief
    /// quiet instants would otherwise fire spurious `beta_peak` bursts; use
    /// 0.3–0.5 s for a walking robot, or 0.0 for the legacy instant behavior.
    ///
    /// Reasonable starting values: `beta_peak = 0.6`, `decay_tau = 0.4`,
    /// `accel_tol_g = 0.15`, `gyro_tol_rad_s = 0.15`, `still_min_s = 0.4`,
    /// base `beta` around `0.1` (or lower for gyro-dominant tracking).
    pub fn set_motion_adaptive(
        &mut self,
        beta_peak: f64,
        decay_tau: f64,
        accel_tol_g: f32,
        gyro_tol_rad_s: f32,
        still_min_s: f64,
    ) {
        self.adaptive = Some(AdaptiveBeta {
            beta_peak,
            decay_tau,
            accel_tol_g,
            gyro_tol_rad_s,
            still_min_s,
        });
    }

    /// Disable motion-adaptive gain; the filter reverts to a constant `beta`.
    pub fn clear_motion_adaptive(&mut self) {
        self.adaptive = None;
    }

    /// Reset the orientation directly from one accelerometer sample (in g),
    /// assumed to be pure gravity — i.e. the device is still. Sets the tilt
    /// exactly (yaw = 0), so the filter starts CONVERGED instead of slewing
    /// from identity at beta-rate. Call once at startup with the first clean
    /// sample (|a| ≈ 1 g); essential with a low base beta, where convergence
    /// from identity would otherwise take tens of seconds.
    pub fn reset_orientation_from_accel(&mut self, accel: [f32; 3]) {
        let a = Vector3::new(accel[0] as f64, accel[1] as f64, accel[2] as f64);
        if a.norm() < 1e-6 {
            return;
        }
        // Want q (body→world) with q · a_norm = world up [0,0,1] — the same
        // convention update_imu converges to (at rest the accel reads +1 g "up").
        let q = nalgebra::UnitQuaternion::rotation_between(&a, &Vector3::z())
            .unwrap_or_else(|| {
                nalgebra::UnitQuaternion::from_axis_angle(
                    &Vector3::x_axis(),
                    core::f64::consts::PI,
                )
            });
        self.filter = Madgwick::new_with_quat(1.0 / 104.0, self.beta, q);
        self.time_still = 0.0;
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
                    if self.time_still >= a.still_min_s {
                        // Boost engages only after sustained stillness; the decay
                        // clock starts at the moment the boost engages.
                        let boost_t = self.time_still - a.still_min_s;
                        self.beta + (a.beta_peak - self.beta) * (-boost_t / a.decay_tau).exp()
                    } else {
                        self.beta
                    }
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
        still_min_s: f64,
    ) {
        self.ahrs
            .set_motion_adaptive(beta_peak, decay_tau, accel_tol_g, gyro_tol_rad_s, still_min_s);
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
