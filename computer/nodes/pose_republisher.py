#!/usr/bin/env python3
"""
ROS 2 Node: Pose Republisher
- Đọc TF map→base_link (do SLAM Toolbox publish)
- Publish lại dưới dạng /robot_pose (PoseStamped) để web dashboard dùng
- Chạy trên máy tính chính (Ubuntu VM/Mac)
"""
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped


class PoseRepublisher(Node):
    def __init__(self):
        super().__init__('pose_republisher')
        self.target_frame = self.declare_parameter('target_frame', 'map').value
        self.source_frame = self.declare_parameter('source_frame', 'base_link').value
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub   = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'Chưa có TF {self.source_frame}->{self.target_frame}: {ex}',
                throttle_duration_sec=5.0)
            return

        msg = PoseStamped()
        msg.header.stamp    = t.header.stamp
        msg.header.frame_id = self.target_frame
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PoseRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
