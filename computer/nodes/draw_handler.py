#!/usr/bin/env python3
"""
ROS 2 Node: Draw Handler
- Subscribe /nav/draw_path (Path) — đường vẽ tay từ web dashboard
- Lọc bớt điểm (lấy mỗi 5 điểm 1) để Nav2 không bị quá tải
- Gửi sang Nav2 action NavigateThroughPoses
- Chạy trên máy tính chính (Ubuntu VM/Mac)
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateThroughPoses


class DrawHandler(Node):
    def __init__(self):
        super().__init__('draw_handler')
        self._current_goal_handle = None
        self.subscription = self.create_subscription(
            Path, '/nav/draw_path', self.path_callback, 10)
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self.get_logger().info('Draw Handler is ready!')

    def path_callback(self, msg):
        self.get_logger().info(f'Received path with {len(msg.poses)} points!')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Server is not responding!')
            return

        # Huỷ goal cũ nếu đang chạy
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        # Lấy mỗi 5 điểm 1, giữ lại điểm cuối
        filtered_poses = list(msg.poses[::5])
        if msg.poses[-1] not in filtered_poses:
            filtered_poses.append(msg.poses[-1])

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = filtered_poses

        self.get_logger().info(f'Sending {len(filtered_poses)} waypoints to Nav2...')
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        self._current_goal_handle = future.result()
        if not self._current_goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2!')
            self._current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = DrawHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
