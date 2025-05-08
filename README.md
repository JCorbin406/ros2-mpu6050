# ros2-mpu6050

ROS 2 driver for the MPU6050 6-axis Inertial Measurement Unit (IMU), combining a 3-axis accelerometer and 3-axis gyroscope. This package reads sensor data over I2C and publishes it as standard ROS 2 messages.

## Features

- Publishes acceleration and angular velocity
- Supports basic calibration and configurable parameters
- Designed for Raspberry Pi running ROS 2 Jazzy Jalisco
- Easily integrable into larger robotic systems

---

## Installation

Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:JCorbin406/ros2-mpu6050.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Hardware Setup

- Connect the MPU6050 to your Raspberry Pi using the I2C interface
- Default I2C address: `0x68`
- Enable I2C on your Pi via `raspi-config` or `modprobe i2c-dev`

---

## Usage

### Launch the node

```bash
ros2 launch mpu6050 mpu6050.launch.py
```

### Parameters (in `config/mpu6050.yaml`)

- `i2c_bus` (int): I2C bus number (e.g., `1` for `/dev/i2c-1`)
- `i2c_address` (int): I2C address of the sensor (`104` for `0x68`)
- `frame_id` (str): Frame name for the IMU data
- `publish_rate` (float): Data publish frequency (Hz)

---

## Topics

- `/mpu6050/imu` (`sensor_msgs/msg/Imu`): IMU data including acceleration and angular velocity

---

## Example Output

```bash
$ ros2 topic echo /mpu6050/imu
linear_acceleration:
  x: -0.03
  y: 9.78
  z: 0.12
angular_velocity:
  x: 0.01
  y: -0.02
  z: 0.00
```

---

## License

Licensed under the [Apache License 2.0](LICENSE).

---

## Author

Jack Corbin — [JCorbin406](https://github.com/JCorbin406)
