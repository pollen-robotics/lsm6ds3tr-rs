//! Driver for the STMicroelectronics LSM6DS3TR-C 6-axis IMU (accelerometer + gyroscope).
//!
//! Uses the `embedded-hal` I2C trait, making it portable across any HAL.
//!
//! Unlike the BMI088 (two separate I2C devices), the LSM6DS3TR-C is a single
//! package exposing accelerometer and gyroscope behind one I2C address, with
//! contiguous output registers and register auto-increment (IF_INC).
//!
//! # Example
//! ```ignore
//! let config = Config::default(); // ±4 g / 104 Hz accel, ±500 °/s / 104 Hz gyro
//! let mut imu = Lsm6ds3trAhrs::new(Lsm6ds3tr::new(i2c, config)?, 0.1);
//!
//! let (ax, ay, az) = imu.imu().read_accelerometer()?;   // g
//! let (gx, gy, gz) = imu.imu().read_gyroscope()?;        // rad/s
//! let temp = imu.imu().read_temperature()?;              // °C
//! ```

mod fusion;
pub use fusion::{Lsm6ds3trAhrs, MadgwickAhrs};

use embedded_hal::i2c::I2c;

// Default 7-bit I2C address (SDO/SA0 low). With SA0 tied high it becomes 0x6B.
pub const DEFAULT_ADDRESS: u8 = 0x6A;

// Accepted WHO_AM_I values. The LSM6DS3TR-C reports 0x6A; the closely related
// LSM6DS3 / LSM6DS3-H report 0x69. All three share the same register map, so
// this driver works against any of them.
const WHO_AM_I_LSM6DS3TR_C: u8 = 0x6A;
const WHO_AM_I_LSM6DS3: u8 = 0x69;

// Registers
const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1_XL: u8 = 0x10; // accelerometer: ODR | FS
const REG_CTRL2_G: u8 = 0x11; // gyroscope: ODR | FS
const REG_CTRL3_C: u8 = 0x12; // BDU, IF_INC, ...
const REG_OUT_TEMP_L: u8 = 0x20;
const REG_OUTX_L_G: u8 = 0x22; // gyroscope X..Z, 6 bytes
const REG_OUTX_L_XL: u8 = 0x28; // accelerometer X..Z, 6 bytes

// CTRL3_C: BDU (block data update) | IF_INC (register auto-increment)
const CTRL3_C_BDU: u8 = 1 << 6;
const CTRL3_C_IF_INC: u8 = 1 << 2;

// ── Configuration enums ──────────────────────────────────────────────────────

/// Accelerometer measurement range.
///
/// Note the unusual register encoding (see datasheet Table 51):
/// `00 = ±2 g`, `01 = ±16 g`, `10 = ±4 g`, `11 = ±8 g`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccRange {
    /// ±2 g
    G2,
    /// ±4 g
    G4,
    /// ±8 g
    G8,
    /// ±16 g
    G16,
}

impl AccRange {
    /// FS_XL bits positioned in CTRL1_XL bits [3:2].
    fn fs_bits(self) -> u8 {
        match self {
            AccRange::G2 => 0b00 << 2,
            AccRange::G16 => 0b01 << 2,
            AccRange::G4 => 0b10 << 2,
            AccRange::G8 => 0b11 << 2,
        }
    }

    /// Sensitivity in mg/LSB (datasheet mechanical characteristics).
    pub fn mg_per_lsb(self) -> f32 {
        match self {
            AccRange::G2 => 0.061,
            AccRange::G4 => 0.122,
            AccRange::G8 => 0.244,
            AccRange::G16 => 0.488,
        }
    }
}

/// Gyroscope measurement range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GyroRange {
    /// ±125 °/s
    Dps125,
    /// ±250 °/s
    Dps250,
    /// ±500 °/s
    Dps500,
    /// ±1000 °/s
    Dps1000,
    /// ±2000 °/s
    Dps2000,
}

impl GyroRange {
    /// FS_G + FS_125 bits positioned in CTRL2_G bits [3:1].
    ///
    /// FS_125 (bit 1) takes precedence; otherwise FS_G[1:0] sit in bits [3:2].
    fn fs_bits(self) -> u8 {
        match self {
            GyroRange::Dps125 => 1 << 1, // FS_125 = 1
            GyroRange::Dps250 => 0b00 << 2,
            GyroRange::Dps500 => 0b01 << 2,
            GyroRange::Dps1000 => 0b10 << 2,
            GyroRange::Dps2000 => 0b11 << 2,
        }
    }

    /// Sensitivity in mdps/LSB (datasheet mechanical characteristics).
    pub fn mdps_per_lsb(self) -> f32 {
        match self {
            GyroRange::Dps125 => 4.375,
            GyroRange::Dps250 => 8.75,
            GyroRange::Dps500 => 17.50,
            GyroRange::Dps1000 => 35.0,
            GyroRange::Dps2000 => 70.0,
        }
    }
}

/// Output data rate, shared by the accelerometer and gyroscope.
///
/// The register encoding (ODR_XL[3:0] / ODR_G[3:0]) is identical for both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Odr {
    Hz12_5,
    Hz26,
    Hz52,
    Hz104,
    Hz208,
    Hz416,
    Hz833,
    Hz1660,
    Hz3330,
    Hz6660,
}

impl Odr {
    /// ODR bits positioned in CTRL1_XL / CTRL2_G bits [7:4].
    fn odr_bits(self) -> u8 {
        let code: u8 = match self {
            Odr::Hz12_5 => 0b0001,
            Odr::Hz26 => 0b0010,
            Odr::Hz52 => 0b0011,
            Odr::Hz104 => 0b0100,
            Odr::Hz208 => 0b0101,
            Odr::Hz416 => 0b0110,
            Odr::Hz833 => 0b0111,
            Odr::Hz1660 => 0b1000,
            Odr::Hz3330 => 0b1001,
            Odr::Hz6660 => 0b1010,
        };
        code << 4
    }
}

// ── Config ───────────────────────────────────────────────────────────────────

/// Sensor configuration passed to [`Lsm6ds3tr::new`].
pub struct Config {
    pub acc_range: AccRange,
    pub acc_odr: Odr,
    pub gyro_range: GyroRange,
    pub gyro_odr: Odr,
}

impl Default for Config {
    /// ±4 g / 104 Hz accelerometer, ±500 °/s / 104 Hz gyroscope.
    fn default() -> Self {
        Config {
            acc_range: AccRange::G4,
            acc_odr: Odr::Hz104,
            gyro_range: GyroRange::Dps500,
            gyro_odr: Odr::Hz104,
        }
    }
}

// ── Error ────────────────────────────────────────────────────────────────────

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    /// WHO_AM_I returned an unexpected value (`found`). The device is not an
    /// LSM6DS3TR-C, or is at a different address.
    WrongId(u8),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

// ── Axis remapping ───────────────────────────────────────────────────────────

/// Software axis remapping applied after reading raw sensor data.
///
/// Defines a permutation and optional sign flip for each output axis,
/// compensating for the physical mounting orientation of the IMU on the board.
///
/// # Example — rotate 90° clockwise around Z (viewed from top):
/// ```
/// // Sensor X → output -Y, Sensor Y → output X, Sensor Z → output Z
/// let remap = AxisRemap { axes: [1, 0, 2], signs: [1.0, -1.0, 1.0] };
/// ```
#[derive(Debug, Clone, Copy)]
pub struct AxisRemap {
    /// Source axis index for each output axis `[x_out, y_out, z_out]`.
    /// `0` = sensor X, `1` = sensor Y, `2` = sensor Z.
    pub axes: [usize; 3],
    /// Sign multiplier for each output axis (`1.0` or `-1.0`).
    pub signs: [f32; 3],
}

impl Default for AxisRemap {
    /// Identity mapping — axes unchanged, no sign flips.
    fn default() -> Self {
        Self {
            axes: [0, 1, 2],
            signs: [1.0, 1.0, 1.0],
        }
    }
}

impl AxisRemap {
    /// Apply the remapping to a raw `(x, y, z)` reading.
    pub fn apply(&self, v: (f32, f32, f32)) -> (f32, f32, f32) {
        let arr = [v.0, v.1, v.2];
        (
            self.signs[0] * arr[self.axes[0]],
            self.signs[1] * arr[self.axes[1]],
            self.signs[2] * arr[self.axes[2]],
        )
    }
}

// ── Transport-free decoder ────────────────────────────────────────────────────

/// A decoded IMU sample in physical units (after axis remap + gyro-bias).
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    /// Gyroscope `[x, y, z]` in **rad/s**.
    pub gyro_rads: [f32; 3],
    /// Accelerometer `[x, y, z]` in **g**.
    pub accel_g: [f32; 3],
}

/// Turns raw output-register bytes into physical, remapped, bias-corrected
/// samples — using the exact same math as [`Lsm6ds3tr`], but with **no I2C and
/// no transport**.
///
/// Use this when the raw registers arrive from somewhere other than this
/// driver's own I2C bus (e.g. relayed over a Dynamixel bus). Pair it with
/// [`MadgwickAhrs`] to reproduce the on-chip [`Lsm6ds3trAhrs`] pipeline byte for
/// byte.
#[derive(Debug, Clone, Copy)]
pub struct SampleDecoder {
    pub acc_range: AccRange,
    pub gyro_range: GyroRange,
    /// Software axis remap applied to every reading.
    pub axis_remap: AxisRemap,
    /// Gyro zero-rate bias in °/s (raw sensor frame), subtracted before remap.
    pub gyro_bias_dps: (f32, f32, f32),
}

impl SampleDecoder {
    /// Decoder with identity remap and zero bias for the given ranges.
    pub fn new(acc_range: AccRange, gyro_range: GyroRange) -> Self {
        Self {
            acc_range,
            gyro_range,
            axis_remap: AxisRemap::default(),
            gyro_bias_dps: (0.0, 0.0, 0.0),
        }
    }

    /// Decode 6 accelerometer bytes `[xl,xh,yl,yh,zl,zh]` → `(x,y,z)` in g.
    pub fn decode_accelerometer(&self, b: &[u8; 6]) -> (f32, f32, f32) {
        let scale = self.acc_range.mg_per_lsb() / 1000.0; // g per LSB
        let x = i16::from_le_bytes([b[0], b[1]]) as f32 * scale;
        let y = i16::from_le_bytes([b[2], b[3]]) as f32 * scale;
        let z = i16::from_le_bytes([b[4], b[5]]) as f32 * scale;
        self.axis_remap.apply((x, y, z))
    }

    /// Decode 6 gyroscope bytes `[xl,xh,yl,yh,zl,zh]` → `(x,y,z)` in °/s.
    pub fn decode_gyroscope_dps(&self, b: &[u8; 6]) -> (f32, f32, f32) {
        let scale = self.gyro_range.mdps_per_lsb() / 1000.0; // dps per LSB
        let x = i16::from_le_bytes([b[0], b[1]]) as f32 * scale - self.gyro_bias_dps.0;
        let y = i16::from_le_bytes([b[2], b[3]]) as f32 * scale - self.gyro_bias_dps.1;
        let z = i16::from_le_bytes([b[4], b[5]]) as f32 * scale - self.gyro_bias_dps.2;
        self.axis_remap.apply((x, y, z))
    }

    /// Decode 6 gyroscope bytes → `(x,y,z)` in rad/s.
    pub fn decode_gyroscope(&self, b: &[u8; 6]) -> (f32, f32, f32) {
        let (x, y, z) = self.decode_gyroscope_dps(b);
        let r = core::f32::consts::PI / 180.0;
        (x * r, y * r, z * r)
    }

    /// Decode a 12-byte block ordered **gyro X..Z then accel X..Z** — i.e. the
    /// LSM6DS3TR-C output-register order (`OUTX_L_G..OUTZ_H_XL`, 0x22..0x2D), the
    /// natural layout of a single burst read. Returns gyro in rad/s, accel in g.
    pub fn decode_block(&self, b: &[u8; 12]) -> ImuSample {
        let g = self.decode_gyroscope(&[b[0], b[1], b[2], b[3], b[4], b[5]]);
        let a = self.decode_accelerometer(&[b[6], b[7], b[8], b[9], b[10], b[11]]);
        ImuSample {
            gyro_rads: [g.0, g.1, g.2],
            accel_g: [a.0, a.1, a.2],
        }
    }
}

// ── Driver ───────────────────────────────────────────────────────────────────

/// LSM6DS3TR-C driver.
pub struct Lsm6ds3tr<I2C> {
    i2c: I2C,
    address: u8,
    acc_range: AccRange,
    gyro_range: GyroRange,
    /// Software axis remapping applied to all accelerometer and gyroscope readings.
    axis_remap: AxisRemap,
    /// Gyroscope zero-rate bias in °/s, in the raw sensor frame (before remap).
    /// Subtracted from every gyroscope reading. Set via [`Lsm6ds3tr::calibrate_gyro`].
    gyro_bias_dps: (f32, f32, f32),
}

impl<I2C: I2c> Lsm6ds3tr<I2C> {
    /// Initialise the sensor at the [`DEFAULT_ADDRESS`] (`0x6A`).
    pub fn new(i2c: I2C, config: Config) -> Result<Self, Error<I2C::Error>> {
        Self::new_with_address(i2c, config, DEFAULT_ADDRESS)
    }

    /// Initialise the sensor at an explicit I2C address (`0x6A` with SA0 low,
    /// `0x6B` with SA0 high).
    pub fn new_with_address(
        mut i2c: I2C,
        config: Config,
        address: u8,
    ) -> Result<Self, Error<I2C::Error>> {
        // Verify identity before touching control registers.
        let mut id = [0u8; 1];
        i2c.write_read(address, &[REG_WHO_AM_I], &mut id)?;
        if id[0] != WHO_AM_I_LSM6DS3TR_C && id[0] != WHO_AM_I_LSM6DS3 {
            return Err(Error::WrongId(id[0]));
        }

        // Block data update + register auto-increment, so a multi-byte read
        // returns a coherent sample across the L/H byte pair.
        i2c.write(address, &[REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC])?;

        // Accelerometer: ODR | FS
        i2c.write(
            address,
            &[REG_CTRL1_XL, config.acc_odr.odr_bits() | config.acc_range.fs_bits()],
        )?;

        // Gyroscope: ODR | FS (+ FS_125)
        i2c.write(
            address,
            &[REG_CTRL2_G, config.gyro_odr.odr_bits() | config.gyro_range.fs_bits()],
        )?;

        Ok(Lsm6ds3tr {
            i2c,
            address,
            acc_range: config.acc_range,
            gyro_range: config.gyro_range,
            axis_remap: AxisRemap::default(),
            gyro_bias_dps: (0.0, 0.0, 0.0),
        })
    }

    /// Set software axis remapping applied to all sensor readings.
    ///
    /// Use this to compensate for the physical mounting orientation of the IMU.
    pub fn set_axis_remap(&mut self, remap: AxisRemap) {
        self.axis_remap = remap;
    }

    /// Snapshot this driver's ranges, remap and gyro bias as a transport-free
    /// [`SampleDecoder`] — decode raw registers obtained over any transport with
    /// exactly the math this driver uses.
    pub fn decoder(&self) -> SampleDecoder {
        SampleDecoder {
            acc_range: self.acc_range,
            gyro_range: self.gyro_range,
            axis_remap: self.axis_remap,
            gyro_bias_dps: self.gyro_bias_dps,
        }
    }

    /// Read accelerometer. Returns `(x, y, z)` in **g** (after axis remapping).
    pub fn read_accelerometer(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(self.address, &[REG_OUTX_L_XL], &mut buf)?;
        Ok(self.decoder().decode_accelerometer(&buf))
    }

    /// Read accelerometer. Returns `(x, y, z)` in **m/s²** (after axis remapping).
    pub fn read_accelerometer_ms2(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_accelerometer()?;
        Ok((x * 9.80665, y * 9.80665, z * 9.80665))
    }

    /// Read gyroscope. Returns `(x, y, z)` in **rad/s** (after axis remapping).
    pub fn read_gyroscope(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_gyroscope_dps()?;
        let to_rad = core::f32::consts::PI / 180.0;
        Ok((x * to_rad, y * to_rad, z * to_rad))
    }

    /// Read gyroscope. Returns `(x, y, z)` in **degrees/s** (after axis remapping).
    pub fn read_gyroscope_dps(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(self.address, &[REG_OUTX_L_G], &mut buf)?;
        Ok(self.decoder().decode_gyroscope_dps(&buf))
    }

    /// Measure and store the gyroscope zero-rate bias.
    ///
    /// **The device must be held completely still during this call.** It reads
    /// `samples` gyroscope measurements, averages them, and stores the result as
    /// a bias that is subtracted from all subsequent gyroscope readings. This is
    /// the main remedy for yaw drift on a 6-axis IMU, where the gyro Z bias has
    /// no gravity/magnetometer reference to correct it.
    ///
    /// The bias is captured in the raw sensor frame, so it is correct regardless
    /// of any [`AxisRemap`] configured before or after calibration.
    pub fn calibrate_gyro(&mut self, samples: u32) -> Result<(), Error<I2C::Error>> {
        if samples == 0 {
            return Ok(());
        }
        // Read without the current bias applied.
        self.gyro_bias_dps = (0.0, 0.0, 0.0);
        let saved_remap = self.axis_remap;
        self.axis_remap = AxisRemap::default();

        let mut sum = (0.0f32, 0.0f32, 0.0f32);
        for _ in 0..samples {
            let (x, y, z) = self.read_gyroscope_dps()?;
            sum.0 += x;
            sum.1 += y;
            sum.2 += z;
        }

        let n = samples as f32;
        self.gyro_bias_dps = (sum.0 / n, sum.1 / n, sum.2 / n);
        self.axis_remap = saved_remap;
        Ok(())
    }

    /// Current gyroscope zero-rate bias in °/s (raw sensor frame).
    pub fn gyro_bias_dps(&self) -> (f32, f32, f32) {
        self.gyro_bias_dps
    }

    /// Read temperature. Returns value in **°C**.
    ///
    /// 16-bit two's complement, 256 LSB/°C with a 25 °C offset.
    pub fn read_temperature(&mut self) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c.write_read(self.address, &[REG_OUT_TEMP_L], &mut buf)?;

        let raw = i16::from_le_bytes([buf[0], buf[1]]);
        Ok(raw as f32 / 256.0 + 25.0)
    }

    /// Consume the driver and return the underlying I2C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn le(v: i16) -> [u8; 2] {
        v.to_le_bytes()
    }

    #[test]
    fn decode_block_scales_and_orders_correctly() {
        // Default ranges: ±4 g (0.122 mg/LSB), ±500 °/s (17.5 mdps/LSB).
        let dec = SampleDecoder::new(AccRange::G4, GyroRange::Dps500);

        let gx = (100.0 / 0.0175) as i16; // ~100 °/s on X
        let az = (1.0 / 0.000122) as i16; // ~1 g on Z
        let mut b = [0u8; 12];
        b[0..2].copy_from_slice(&le(gx)); // gyro X (block is gyro first)
        b[10..12].copy_from_slice(&le(az)); // accel Z (bytes 6..11 are accel)

        let s = dec.decode_block(&b);
        // gyro X ≈ 100 °/s in rad/s
        assert!((s.gyro_rads[0] - 100.0 * core::f32::consts::PI / 180.0).abs() < 0.01);
        assert!(s.gyro_rads[1].abs() < 1e-3 && s.gyro_rads[2].abs() < 1e-3);
        // accel = gravity on +Z
        assert!(s.accel_g[0].abs() < 1e-3 && s.accel_g[1].abs() < 1e-3);
        assert!((s.accel_g[2] - 1.0).abs() < 1e-3);
    }

    #[test]
    fn decoder_matches_axis_remap_and_bias() {
        let mut dec = SampleDecoder::new(AccRange::G4, GyroRange::Dps500);
        // Invert X and Y (the microduck on-board convention).
        dec.axis_remap = AxisRemap { axes: [0, 1, 2], signs: [-1.0, -1.0, 1.0] };

        let g = (200.0_f32 / 0.0175) as i16; // 200 °/s on X and Y
        let mut b = [0u8; 12];
        b[0..2].copy_from_slice(&le(g));
        b[2..4].copy_from_slice(&le(g));
        let s = dec.decode_block(&b);
        // X and Y signs flipped by the remap.
        assert!(s.gyro_rads[0] < 0.0 && s.gyro_rads[1] < 0.0);
    }
}
