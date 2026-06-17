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
pub use fusion::Lsm6ds3trAhrs;

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
    fn mg_per_lsb(self) -> f32 {
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
    fn mdps_per_lsb(self) -> f32 {
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

// ── Driver ───────────────────────────────────────────────────────────────────

/// LSM6DS3TR-C driver.
pub struct Lsm6ds3tr<I2C> {
    i2c: I2C,
    address: u8,
    acc_range: AccRange,
    gyro_range: GyroRange,
    /// Software axis remapping applied to all accelerometer and gyroscope readings.
    axis_remap: AxisRemap,
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
        })
    }

    /// Set software axis remapping applied to all sensor readings.
    ///
    /// Use this to compensate for the physical mounting orientation of the IMU.
    pub fn set_axis_remap(&mut self, remap: AxisRemap) {
        self.axis_remap = remap;
    }

    /// Read accelerometer. Returns `(x, y, z)` in **g** (after axis remapping).
    pub fn read_accelerometer(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(self.address, &[REG_OUTX_L_XL], &mut buf)?;

        let scale = self.acc_range.mg_per_lsb() / 1000.0; // g per LSB
        let x = i16::from_le_bytes([buf[0], buf[1]]) as f32 * scale;
        let y = i16::from_le_bytes([buf[2], buf[3]]) as f32 * scale;
        let z = i16::from_le_bytes([buf[4], buf[5]]) as f32 * scale;
        Ok(self.axis_remap.apply((x, y, z)))
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

        let scale = self.gyro_range.mdps_per_lsb() / 1000.0; // dps per LSB
        let x = i16::from_le_bytes([buf[0], buf[1]]) as f32 * scale;
        let y = i16::from_le_bytes([buf[2], buf[3]]) as f32 * scale;
        let z = i16::from_le_bytes([buf[4], buf[5]]) as f32 * scale;
        Ok(self.axis_remap.apply((x, y, z)))
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
