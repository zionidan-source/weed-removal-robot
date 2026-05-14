#!/usr/bin/env python3
"""
UR5 Node
========
Bridges the vision coordinator and MoveIt2.
Subscribes to /coordinator/target, plans a trajectory with MoveIt2,
optionally asks for manual confirmation, executes, then publishes status.

Subscriptions:
    /coordinator/target  (geometry_msgs/PointStamped)  — 3D target in base_link frame

Publications:
    /ur5/status          (std_msgs/String)              — "done" or "error"

Parameters (see ur5_params.yaml):
    require_confirmation          (bool, default True)  — prompt before executing
    allowed_planning_time         (float, default 5.0)
    num_planning_attempts         (int,   default 5)
    max_velocity_scaling_factor   (float, default 0.1)
    max_acceleration_scaling_factor (float, default 0.1)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints, PositionConstraint, MoveItErrorCodes, DisplayTrajectory
)
from geometry_msgs.msg import PointStamped, PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


class UR5Node(Node):

    def __init__(self):
        super().__init__('ur5_node')
        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('require_confirmation', True)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('max_velocity_scaling_factor', 0.1)
        self.declare_parameter('max_acceleration_scaling_factor', 0.1)

        self.require_confirmation = self.get_parameter('require_confirmation') \
            .get_parameter_value().bool_value
        self.allowed_planning_time = self.get_parameter('allowed_planning_time') \
            .get_parameter_value().double_value
        self.num_planning_attempts = self.get_parameter('num_planning_attempts') \
            .get_parameter_value().integer_value
        self.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling_factor') \
            .get_parameter_value().double_value
        self.max_acceleration_scaling_factor = self.get_parameter('max_acceleration_scaling_factor') \
            .get_parameter_value().double_value

        # ── MoveIt2 action clients ───────────────────────────────────
        self.movegroup_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.cb_group)
        self.exec_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self.cb_group)

        # ── RViz visualization ───────────────────────────────────────
        self.display_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path', 10)

        # ── Status feedback to coordinator ───────────────────────────
        self.status_pub = self.create_publisher(
            String, '/ur5/status', 10)

        # ── Subscriber: target from coordinator ──────────────────────
        self.create_subscription(
            PointStamped,
            '/coordinator/target',
            self._target_callback, 10,
            callback_group=self.cb_group)

        # ── MoveIt2 config ───────────────────────────────────────────
        self.group_name = 'ur_manipulator'
        self.base_frame = 'base_link'
        self.ee_link    = 'tool0'

        self.busy = False

        self.get_logger().info(
            f'UR5 node ready (require_confirmation={self.require_confirmation}). '
            'Waiting for targets on /coordinator/target ...')

    # ─────────────────────────────────────────────────────────────────
    async def _target_callback(self, msg: PointStamped):
        """Called when coordinator sends a new weed coordinate."""

        if self.busy:
            self.get_logger().warn(
                'Received new target while arm is busy — ignoring. '
                'Coordinator should wait for "done" before sending next.')
            return

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        self.get_logger().info(
            f'Target received: ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'in frame "{msg.header.frame_id}"')

        self.busy = True
        result = await self._plan_confirm_execute(x, y, z)
        self.busy = False

        status_msg = String()
        if result is True:
            status_msg.data = 'done'
            self.get_logger().info('Move complete — publishing "done"')
        elif result == 'declined':
            status_msg.data = 'declined'
            self.get_logger().info('Execution declined — publishing "declined"')
        else:
            status_msg.data = 'error'
            self.get_logger().warn('Move failed — publishing "error"')

        self.status_pub.publish(status_msg)

    # ─────────────────────────────────────────────────────────────────
    async def _plan_confirm_execute(self, x, y, z):
        """
        Plan with MoveIt2, show in RViz, optionally confirm, then execute.
        Returns True on success, 'declined' if user pressed n, False on failure.
        """

        # ── Wait for MoveIt2 servers ─────────────────────────────────
        self.get_logger().info('Waiting for MoveIt2 servers...')
        if not self.movegroup_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available!')
            return False
        if not self.exec_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ExecuteTrajectory action server not available!')
            return False

        # ── Step 1: Plan only ────────────────────────────────────────
        plan_goal = self._build_movegroup_goal(x, y, z, plan_only=True)
        self.get_logger().info(f'Planning to: ({x:.3f}, {y:.3f}, {z:.3f}) ...')

        goal_handle = await self.movegroup_client.send_goal_async(plan_goal)

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected')
            return False

        mg_result_response = await goal_handle.get_result_async()
        mg_result = mg_result_response.result
        err = mg_result.error_code.val

        if err != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'Planning failed — MoveIt error code: {err}')
            return False

        planned_traj = mg_result.planned_trajectory
        start_state  = mg_result.trajectory_start

        # ── Step 2: Publish to RViz ──────────────────────────────────
        disp = DisplayTrajectory()
        disp.trajectory_start = start_state
        disp.trajectory.append(planned_traj)
        self.display_pub.publish(disp)
        self.get_logger().info(
            'Plan found. Check RViz (/display_planned_path) to preview.')

        # ── Step 3: Confirmation ─────────────────────────────────────
        if self.require_confirmation:
            self.get_logger().info(
                f'Target: x={x:.3f}  y={y:.3f}  z={z:.3f}')
            ans = input('  Execute this trajectory? [y/N]: ').strip().lower()
            if ans not in ('y', 'yes'):
                self.get_logger().info('Execution cancelled by user.')
                return 'declined'

        # ── Step 4: Execute trajectory ───────────────────────────────
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = planned_traj

        self.get_logger().info('Executing trajectory...')
        exec_handle = await self.exec_client.send_goal_async(exec_goal)

        if not exec_handle or not exec_handle.accepted:
            self.get_logger().error('ExecuteTrajectory goal rejected')
            return False

        exec_res_response = await exec_handle.get_result_async()
        exec_result = exec_res_response.result
        exec_err = exec_result.error_code.val

        if exec_err == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Execution succeeded.')
            return True
        else:
            self.get_logger().error(
                f'Execution failed — MoveIt error code: {exec_err}')
            return False

    # ─────────────────────────────────────────────────────────────────
    def _build_movegroup_goal(self, x, y, z, plan_only: bool) -> MoveGroup.Goal:
        """Build a MoveGroup goal for a position target."""
        goal = MoveGroup.Goal()
        req  = goal.request

        req.group_name                      = self.group_name
        req.allowed_planning_time           = self.allowed_planning_time
        req.num_planning_attempts           = self.num_planning_attempts
        req.max_velocity_scaling_factor     = self.max_velocity_scaling_factor
        req.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor

        # Position constraint: 5 mm sphere around the target
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name       = self.ee_link

        sphere = SolidPrimitive()
        sphere.type       = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]

        pose = PoseStamped()
        pose.header.frame_id    = self.base_frame
        pose.pose.position.x    = x
        pose.pose.position.y    = y
        pose.pose.position.z    = z
        pose.pose.orientation.w = 1.0

        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        req.goal_constraints.append(constraints)

        goal.planning_options.plan_only = plan_only

        return goal


# ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = UR5Node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
