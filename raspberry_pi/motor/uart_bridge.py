#!/usr/bin/env python3
"""
ROS 2 Node: UART Bridge (Pi → STM32)
- Subscribe /cmd_vel (Twist) → chuyển thành lệnh text → gửi STM32 qua /dev/ttyAMA0
- Subscribe /robot_mode (String) → bật/tắt chế độ AUTO
- Tự động reconnect khi mất kết nối serial
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time

LIN_THRESH  = 0.05   # m/s — ngưỡng tiến/lùi, khớp nav2_params controller_server
ANG_THRESH  = 0.15   # rad/s — ngưỡng xoay, khớp nav2_params controller_server
SERIAL_PORT = '/dev/ttyAMA0'
BAUD        = 115200


def twist_to_cmd(lx: float, az: float) -> str:
    """Chuyển Twist sang lệnh text cho STM32 (8 hướng + dừng)."""
    fwd  = lx >  LIN_THRESH
    back = lx < -LIN_THRESH
    lft  = az >  ANG_THRESH
    rgt  = az < -ANG_THRESH

    if fwd  and lft: return 'UP_LEFT'
    if fwd  and rgt: return 'UP_RIGHT'
    if fwd:          return 'ON'
    if back and lft: return 'DOWN_LEFT'
    if back and rgt: return 'DOWN_RIGHT'
    if back:         return 'BACK'
    if lft:          return 'LEFT'
    if rgt:          return 'RIGHT'
    return 'OFF'


class UartBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')
        self._last_cmd  = ''
        self._auto_mode = False
        self.ser        = None
        self._open_serial(initial=True)
        self._reconnect_timer = self.create_timer(2.0, self._check_serial)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(Twist,  '/cmd_vel',    self._on_vel,  qos)
        self.create_subscription(String, '/robot_mode', self._on_mode, 10)

    def _open_serial(self, initial: bool = False) -> bool:
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
            time.sleep(2.0 if initial else 0.2)
            self._last_cmd = ''
            return True
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f'Serial Error: {e}')
            return False

    def _check_serial(self):
        if self.ser is None or not self.ser.is_open:
            self._open_serial(initial=False)

    def _write(self, cmd: str):
        if cmd == self._last_cmd:
            return
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write(f'{cmd}\n'.encode('utf-8'))
            self.ser.flush()
            self._last_cmd = cmd
        except Exception as e:
            self.get_logger().error(f'Write error: {e}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _on_vel(self, msg: Twist):
        if self._auto_mode:
            return
        self._write(twist_to_cmd(msg.linear.x, msg.angular.z))

    def _on_mode(self, msg: String):
        mode = msg.data.strip().upper()
        if mode == 'AUTO':
            self._auto_mode = True
            self._write('AUTO')
        else:
            self._auto_mode = False
            self._write('OFF')

    def stop_robot(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'OFF\n')
                self.ser.flush()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = UartBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
