# mpu6050 - ROS 2 IMU Interface Node

This ROS 2 package provides a Python-based node that interfaces with the MPU6050 accelerometer and gyroscope sensor over I2C. It publishes filtered and bias-calibrated IMU data to the `/imu` topic using `sensor_msgs/msg/Imu`.

---

## Features

- Communicates with MPU6050 via I2C
- Publishes accelerometer and gyroscope data in SI units
- Applies Butterworth low-pass filtering to reduce noise
- Supports calibration to remove sensor bias while preserving gravity
- Fully configurable via YAML and launch files
- Compatible with Raspberry Pi (tested on Pi 4 with ROS 2 Jazzy)

---

## Installation

### Clone into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/JCorbin406/mpu6050.git
```
### Build the workspace

```bash
cd ~/ros2_ws
colcon build --packages-select mpu6050
source install/setup.bash
```

---

## Usage

### Launch the IMU node with default parameters:

```bash
ros2 launch mpu6050 mpu6050.launch.py
```

This loads configuration from:
```
mpu6050/config/mpu6050_params.yaml
```

You can also manually run the node:

```bash
ros2 run mpu6050 mpu_node
```

---

## Parameters

You can customize behavior via the YAML config file:

```yaml
mpu6050_node:
  ros__parameters:
    i2c_bus: 1               # I2C bus (typically 1 on Raspberry Pi)
    i2c_address: 104         # Decimal address (104 = 0x68)
    accel_range: 8           # 2, 4, 8, or 16 (g)
    gyro_range: 500          # 250, 500, 1000, or 2000 (deg/s)
    calibration_samples: 100 # Number of samples to average during startup
    publish_rate: 50.0       # Hz
```

---

## Topic

The node publishes:

| Topic | Type | Description |
|-------|------|-------------|
| `/imu` | `sensor_msgs/msg/Imu` | Filtered, bias-corrected IMU data |

> Note: Orientation and covariance values are not provided by the MPU6050 directly.

---

## File Structure

```
mpu6050/
├── config/
│   └── mpu6050_params.yaml
├── launch/
│   └── mpu6050.launch.py
├── mpu6050/
│   ├── __init__.py
│   ├── mpu6050.py           # Sensor driver
│   └── mpu_node.py          # ROS 2 node
├── package.xml
└── setup.py
```

---

## Maintainer

Jack Corbin  
[https://github.com/JCorbin406](https://github.com/JCORBIN406)
