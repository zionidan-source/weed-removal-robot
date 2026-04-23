"""
UR5 Control Node
================
Receives pick coordinates from the coordinator and moves the UR5 arm.
Publishes status feedback ("done" / "error") when motion completes.

Subscriptions:
    /coordinator/target  (geometry_msgs/PointStamped)  — 3D coordinate in base frame

Publications:
    /ur5/status          (std_msgs/String)              — "done" or "error"

TODO: Replace the placeholder motion logic below with your actual
      UR5 control code (MoveIt, ur_robot_driver, URScript, etc.)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class UR5Node(Node):

    def __init__(self):
        super().__init__('ur5_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('robot_ip', '192.168.1.100')
        self.declare_parameter('move_speed', 0.25)       # m/s
        self.declare_parameter('move_acceleration', 0.5)  # m/s²

        self.robot_ip = self.get_parameter('robot_ip') \
            .get_parameter_value().string_value
        self.speed = self.get_parameter('move_speed') \
            .get_parameter_value().double_value
        self.accel = self.get_parameter('move_acceleration') \
            .get_parameter_value().double_value

        # ── Publisher: status feedback to coordinator ────────────────
        self.status_pub = self.create_publisher(
            String, '/ur5/status', 10)

        # ── Subscriber: target coordinates from coordinator ─────────
        self.create_subscription(
            PointStamped,
            '/coordinator/target',
            self._target_callback, 10)

        self.get_logger().info(
            f'UR5 node ready. Robot IP: {self.robot_ip}, '
            f'speed: {self.speed} m/s, accel: {self.accel} m/s²')

    def _target_callback(self, msg: PointStamped):
        """Receive a target coordinate and move the arm."""
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        self.get_logger().info(
            f'Received target: ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'in frame "{msg.header.frame_id}"')

        # ────────────────────────────────────────────────────────
        # TODO: REPLACE THIS SECTION WITH YOUR ACTUAL MOTION CODE
        #
        # This is where your existing UR5 control logic goes.
        # For example, using MoveIt2:
        #
        #   pose = Pose()
        #   pose.position.x = x
        #   pose.position.y = y
        #   pose.position.z = z
        #   pose.orientation.w = 1.0  # straight down
        #   self.move_group.set_pose_target(pose)
        #   success = self.move_group.go(wait=True)
        #
        # Or using URScript via ur_robot_driver:
        #
        #   script = f"movel(p[{x}, {y}, {z}, 0, 3.14, 0], "
        #   script += f"a={self.accel}, v={self.speed})"
        #   self.urscript_pub.publish(String(data=script))
        #
        # ────────────────────────────────────────────────────────

        success = self._move_to_point(x, y, z)

        # Publish status feedback
        status_msg = String()
        if success:
            status_msg.data = 'done'
            self.get_logger().info('Move complete — publishing "done"')
        else:
            status_msg.data = 'error'
            self.get_logger().warn('Move failed — publishing "error"')

        self.status_pub.publish(status_msg)

    def _move_to_point(self, x, y, z):
        """
        Move the UR5 to the given (x, y, z) coordinate.

        TODO: Replace this placeholder with your actual implementation.
        This currently just logs and returns True.

        Returns:
            bool: True if motion succeeded, False if it failed.
        """
        self.get_logger().warn(
            'PLACEHOLDER: _move_to_point() not yet implemented. '
            'Replace this with your UR5 motion code.')

        # Placeholder — simulate success
        return True


# ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = UR5Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
