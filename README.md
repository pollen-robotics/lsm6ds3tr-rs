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

`new` uses the default I2C address `0x6A` (SDO/SA0 low). Use `Lsm6ds3tr::new_with_address(i2c, config, 0x6B)` when SA0 is tied high. Construction reads `WHO_AM_I` and returns `Error::WrongId` if it isn't an LSM6DS3TR-C.

`Config` defaults to ±4 g / 104 Hz accelerometer and ±500 °/s / 104 Hz gyroscope. All ranges and output data rates are configurable via `AccRange`, `GyroRange`, and `Odr`.

The Madgwick `beta` parameter controls convergence speed vs. noise. `0.1` is a good starting point.

## Examples

Both examples target `/dev/i2c-4` at 50 Hz by default.

### read_imu

Print quaternion and temperature in a loop, or stream data over TCP for visualization.

```sh
# Print mode
cargo run --example read_imu
cargo run --example read_imu -- --bus /dev/i2c-1 --freq 100

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
