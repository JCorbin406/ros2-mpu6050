import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050.mpu6050 import mpu6050
import numpy as np
from scipy.signal import butter, lfilter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(Imu, 'imu', imu_qos)

        # Declare tunable parameters
        self.declare_parameter('accel_range', 8)  # default to ±8g
        self.declare_parameter('gyro_range', 500)  # default to ±500°/s
        self.declare_parameter('calibration_samples', 100)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 104)
        self.declare_parameter('publish_rate', 50.0)

        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        i2c_addr = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.sample_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Read the parameters
        accel_range_val = self.get_parameter('accel_range').get_parameter_value().integer_value
        gyro_range_val = self.get_parameter('gyro_range').get_parameter_value().integer_value
        num_calib_samples = self.get_parameter('calibration_samples').get_parameter_value().integer_value

        self.sensor = mpu6050(i2c_addr, bus=i2c_bus)

        # Map input values to actual register settings
        accel_range_map = {
            2: self.sensor.ACCEL_RANGE_2G,
            4: self.sensor.ACCEL_RANGE_4G,
            8: self.sensor.ACCEL_RANGE_8G,
            16: self.sensor.ACCEL_RANGE_16G
        }
        gyro_range_map = {
            250: self.sensor.GYRO_RANGE_250DEG,
            500: self.sensor.GYRO_RANGE_500DEG,
            1000: self.sensor.GYRO_RANGE_1000DEG,
            2000: self.sensor.GYRO_RANGE_2000DEG
        }

        # Apply settings
        self.sensor.set_accel_range(accel_range_map.get(accel_range_val, self.sensor.ACCEL_RANGE_8G))
        self.sensor.set_gyro_range(gyro_range_map.get(gyro_range_val, self.sensor.GYRO_RANGE_500DEG))

        # Display sensor measurement ranges so users can verify accuracy
        accel_range = self.sensor.read_accel_range()
        gyro_range = self.sensor.read_gyro_range()
        self.get_logger().info(f"Accelerometer range: ±{accel_range}g")
        self.get_logger().info(f"Gyroscope range: ±{gyro_range}°/s")

        # Butterworth filter config
        self.filter_order = 2       # 2nd order filter
        self.cutoff_freq = 10.0      # Cutoff frequency in Hz
        self.b, self.a = butter(self.filter_order, self.cutoff_freq / (0.5 * self.sample_rate), btype='low')

        # Buffers for filtering (store last N raw samples)
        self.accel_history = np.zeros((3, 20))  # 3 axes, 20 samples
        self.gyro_history = np.zeros((3, 20))
        self.hist_len = self.accel_history.shape[1]

        self.get_logger().info('Calibrating MPU6050...')
        self.accel_bias, self.gyro_bias = self.calibrate_sensor(num_samples=num_calib_samples)
        self.get_logger().info('Calibration complete.')
        self.get_logger().info('Publishing to topic /imu.')

        self.timer = self.create_timer(1.0 / self.sample_rate, self.read_and_publish)

    def calibrate_sensor(self, num_samples=100):
        accel_samples = []
        gyro_samples = []

        for _ in range(num_samples):
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()
            accel_samples.append([accel['x'], accel['y'], accel['z']])
            gyro_samples.append([gyro['x'], gyro['y'], gyro['z']])
            rclpy.spin_once(self, timeout_sec=0.01)

        accel_avg = np.mean(accel_samples, axis=0)
        gyro_bias = np.mean(gyro_samples, axis=0)

        # Subtract gravity from Z so we preserve it later
        gravity = 9.80665  # Standard gravity in m/s²
        accel_bias = np.array([accel_avg[0], accel_avg[1], accel_avg[2] - gravity])

        return accel_bias, gyro_bias


    def apply_butterworth_filter(self, data_history):
        """Applies a Butterworth filter along each axis."""
        filtered = np.zeros(3)
        for i in range(3):
            filtered[i] = lfilter(self.b, self.a, data_history[i])[self.hist_len - 1]
        return filtered

    def read_and_publish(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        # Apply bias correction
        raw_accel = np.array([
            accel['x'] - self.accel_bias[0],
            accel['y'] - self.accel_bias[1],
            accel['z'] - self.accel_bias[2]
        ])
        raw_gyro = np.array([
            gyro['x'] - self.gyro_bias[0],
            gyro['y'] - self.gyro_bias[1],
            gyro['z'] - self.gyro_bias[2]
        ])

        # Shift old data and add new sample
        self.accel_history = np.roll(self.accel_history, -1, axis=1)
        self.gyro_history = np.roll(self.gyro_history, -1, axis=1)
        self.accel_history[:, -1] = raw_accel
        self.gyro_history[:, -1] = raw_gyro

        # Apply filter
        filtered_accel = self.apply_butterworth_filter(self.accel_history)
        filtered_gyro = self.apply_butterworth_filter(self.gyro_history)

        imu_msg = Imu()
        # Populate ROS message
        imu_msg.linear_acceleration.x = filtered_accel[0]
        imu_msg.linear_acceleration.y = filtered_accel[1]
        imu_msg.linear_acceleration.z = filtered_accel[2]
        imu_msg.angular_velocity.x = filtered_gyro[0]
        imu_msg.angular_velocity.y = filtered_gyro[1]
        imu_msg.angular_velocity.z = filtered_gyro[2]
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        self.publisher_.publish(imu_msg)
        # self.get_logger().info(
        #     f"Accel: ({filtered_accel[0]:.2f}, {filtered_accel[1]:.2f}, {filtered_accel[2]:.2f}) m/s^2 | "
        #     f"Gyro: ({filtered_gyro[0]:.2f}, {filtered_gyro[1]:.2f}, {filtered_gyro[2]:.2f}) deg/s"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
