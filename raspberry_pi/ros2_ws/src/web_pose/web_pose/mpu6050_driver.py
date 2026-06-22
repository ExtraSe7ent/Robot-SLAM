#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import math
import time

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43

GYRO_SCALE = 131.0
DEG2RAD = math.pi / 180.0
SCALE_FACTOR = DEG2RAD / GYRO_SCALE
CALIBRATE_N = 150

class MPU6050Driver(Node):
    def __init__(self):
        super().__init__('mpu6050_driver')
        self.bus = smbus2.SMBus(1)
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)
        time.sleep(0.05)
        self._gz_offset = self._calibrate()
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_timer(0.02, self._publish)
        self.get_logger().info(f'MPU6050 ready: offset={math.degrees(self._gz_offset):.3f} deg/s')

    def _read_i16(self, reg: int) -> int:
        d = self.bus.read_i2c_block_data(MPU6050_ADDR, reg, 2)
        v = (d[0] << 8) | d[1]
        return v - 65536 if v >= 0x8000 else v

    def _calibrate(self) -> float:
        total = 0.0
        for _ in range(CALIBRATE_N):
            total += self._read_i16(GYRO_XOUT_H + 4)
            time.sleep(0.02)
        return (total / CALIBRATE_N) * SCALE_FACTOR

    def _publish(self):
        try:
            d = self.bus.read_i2c_block_data(MPU6050_ADDR, GYRO_XOUT_H, 6)
            v = (d[4] << 8) | d[5]
            gz_raw = v - 65536 if v >= 0x8000 else v
            gz_rads = (gz_raw * SCALE_FACTOR) - self._gz_offset
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.angular_velocity.z = float(gz_rads)
            msg.angular_velocity_covariance = [
                1e6, 0.0, 0.0,
                0.0, 1e6, 0.0,
                0.0, 0.0, 1e-4
            ]
            msg.linear_acceleration_covariance[0] = -1.0
            msg.orientation_covariance[0] = -1.0
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'IMU Error: {e}', throttle_duration_sec=3.0)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
