import smbus2
from time import sleep


class mpu6050:
    # Constants
    GRAVITIY_MS2 = 9.80665

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address, bus=1):
        """Initialize the MPU6050 on a given I2C bus and address."""
        self.address = address
        self.bus = smbus2.SMBus(bus)  # ← now configurable!

        # Wake up the MPU6050 (clears sleep mode bit)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    def read_i2c_word(self, register):
        """Read two bytes and return combined signed value."""
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        return value

    def get_temp(self):
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)
        return (raw_temp / 340.0) + 36.53

    def set_accel_range(self, accel_range):
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw=False):
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
        if raw:
            return raw_data
        return {
            self.ACCEL_RANGE_2G: 2,
            self.ACCEL_RANGE_4G: 4,
            self.ACCEL_RANGE_8G: 8,
            self.ACCEL_RANGE_16G: 16
        }.get(raw_data, -1)

    def get_accel_data(self, g=False):
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_range = self.read_accel_range(raw=True)
        modifier = {
            self.ACCEL_RANGE_2G: self.ACCEL_SCALE_MODIFIER_2G,
            self.ACCEL_RANGE_4G: self.ACCEL_SCALE_MODIFIER_4G,
            self.ACCEL_RANGE_8G: self.ACCEL_SCALE_MODIFIER_8G,
            self.ACCEL_RANGE_16G: self.ACCEL_SCALE_MODIFIER_16G,
        }.get(accel_range, self.ACCEL_SCALE_MODIFIER_2G)

        x /= modifier
        y /= modifier
        z /= modifier

        if g:
            return {'x': x, 'y': y, 'z': z}
        return {
            'x': x * self.GRAVITIY_MS2,
            'y': y * self.GRAVITIY_MS2,
            'z': z * self.GRAVITIY_MS2
        }

    def set_gyro_range(self, gyro_range):
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, raw=False):
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
        if raw:
            return raw_data
        return {
            self.GYRO_RANGE_250DEG: 250,
            self.GYRO_RANGE_500DEG: 500,
            self.GYRO_RANGE_1000DEG: 1000,
            self.GYRO_RANGE_2000DEG: 2000
        }.get(raw_data, -1)

    def get_gyro_data(self):
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_range = self.read_gyro_range(raw=True)
        modifier = {
            self.GYRO_RANGE_250DEG: self.GYRO_SCALE_MODIFIER_250DEG,
            self.GYRO_RANGE_500DEG: self.GYRO_SCALE_MODIFIER_500DEG,
            self.GYRO_RANGE_1000DEG: self.GYRO_SCALE_MODIFIER_1000DEG,
            self.GYRO_RANGE_2000DEG: self.GYRO_SCALE_MODIFIER_2000DEG,
        }.get(gyro_range, self.GYRO_SCALE_MODIFIER_250DEG)

        return {
            'x': x / modifier,
            'y': y / modifier,
            'z': z / modifier
        }

    def get_all_data(self):
        return [self.get_accel_data(), self.get_gyro_data(), self.get_temp()]
