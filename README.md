# lsm6ds3tr-rs

Rust driver for the STMicroelectronics LSM6DS3TR-C 6-axis IMU (accelerometer + gyroscope) over I2C, with a built-in Madgwick filter for orientation estimation.

The API mirrors [bmi088-rs](https://github.com/pollen-robotics/bmi088-rs). The main difference is the hardware: the LSM6DS3TR-C is a single package (one I2C address, contiguous output registers with auto-increment) rather than the BMI088's two separate accelerometer/gyroscope devices.

## Library

The driver is generic over any [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) I2C implementation.

```rust
use lsm6ds3tr::{Lsm6ds3tr, Lsm6ds3trAhrs, Config};

let imu = Lsm6ds3tr::new(i2c, Config::default())?;          // default address 0x6A
let mut ahrs = Lsm6ds3trAhrs::new(imu, /* beta */ 0.1);

// Raw sensor data
let (ax, ay, az) = ahrs.imu().read_accelerometer()?;  // g
let (gx, gy, gz) = ahrs.imu().read_gyroscope()?;      // rad/s
let temp         = ahrs.imu().read_temperature()?;     // °C

// Orientation quaternion [w, x, y, z]
let [w, x, y, z] = ahrs.get_quaternion(dt)?;
```

`new` uses the default I2C address `0x6A` (SDO/SA0 low). Use `Lsm6ds3tr::new_with_address(i2c, config, 0x6B)` when SA0 is tied high. Construction reads `WHO_AM_I` and returns `Error::WrongId` if it doesn't match a known device.

The accepted `WHO_AM_I` values are `0x6A` (LSM6DS3TR-C) and `0x69` (LSM6DS3 / LSM6DS3-H). These parts share the same register map, so the driver works against any of them.

`Config` defaults to ±4 g / 104 Hz accelerometer and ±500 °/s / 104 Hz gyroscope. All ranges and output data rates are configurable via `AccRange`, `GyroRange`, and `Odr`.

The Madgwick `beta` parameter controls convergence speed vs. noise. `0.1` is a good starting point.

### Yaw drift and gyro calibration

This is a 6-axis IMU (no magnetometer). Roll and pitch are corrected against gravity by the Madgwick filter, but **yaw has no absolute reference**, so it slowly drifts as the gyro's zero-rate bias integrates over time. To minimise it, measure and subtract that bias once at startup while the device is held still:

```rust
let mut imu = Lsm6ds3tr::new(i2c, Config::default())?;
imu.calibrate_gyro(500)?;   // keep the IMU still during this call
let mut ahrs = Lsm6ds3trAhrs::new(imu, 0.1);
```

Both examples do this automatically on startup. Note this reduces but cannot fully eliminate yaw drift — absolute heading requires a magnetometer.

### Slow reconvergence after fast motion

The Madgwick filter treats the accelerometer as a gravity reference, which only holds when the device isn't accelerating. During fast motion (especially translation) the accelerometer also senses linear acceleration, so roll/pitch get pulled off; with a low `beta` the estimate then takes several seconds to drift back once the device is still.

Enable **motion-adaptive gain** to fix this — trust the gyro while moving, then snap back to gravity quickly once still:

```rust
let mut ahrs = Lsm6ds3trAhrs::new(imu, 0.1);    // normal base gain while moving
ahrs.set_motion_adaptive(0.6, 0.4, 0.15, 0.15); // beta_peak, decay_tau (s), accel_tol (g), gyro_tol (rad/s)
```

While moving, the base `beta` is used. The instant the device becomes still the gain jumps to `beta_peak` (snapping the estimate back to gravity), then **decays back to the base `beta` over `decay_tau` seconds** — so reconvergence is fast but the steady-state output stays as quiet as the base gain. Keep the base `beta` at the value that already behaves well while moving (e.g. `0.1`); the boost never makes moving behavior worse.

The device is considered "still" when the accelerometer magnitude is within `accel_tol` of 1 g **and** the gyroscope magnitude is below `gyro_tol`; if your platform vibrates (e.g. motors running), loosen those tolerances so "still" is actually detected. Both examples enable this by default.

Tuning: more steady-state noise than you like → lower `beta_peak` or shorten `decay_tau`; too slow to reconverge → raise `beta_peak` or lengthen `decay_tau`.

## Examples

Both examples target `/dev/i2c-1`, address `0x6B`, at 50 Hz by default. Override with `--bus`, `--addr` (hex, e.g. `0x6a`), and `--freq`.

### read_imu

Print quaternion and temperature in a loop, or stream data over TCP for visualization.

```sh
# Print mode
cargo run --example read_imu
cargo run --example read_imu -- --bus /dev/i2c-1 --addr 0x6b --freq 100

# Stream mode (see Visualization below)
cargo run --example read_imu -- --stream
cargo run --example read_imu -- --stream --freq 30
```

### bench_imu

Measure how well the loop holds its target rate and how much CPU it uses.

```sh
cargo run --example bench_imu
cargo run --example bench_imu -- --freq 200
```

## Visualization

`imu_client.py` connects to a running `--stream` server and displays the live orientation using [FramesViewer](https://github.com/apirrone/FramesViewer).

**On the robot**, start the stream:
```sh
cargo run --example read_imu -- --stream --freq 50
```

**On your machine**, install dependencies and run the client:
```sh
pip install -r requirements.txt
python imu_client.py --ip <robot-ip>
```
