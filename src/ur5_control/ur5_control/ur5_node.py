#!/usr/bin/env python3
"""
UR5 Coordinator Node
====================
Bridges the vision coordinator and MoveIt2.
Subscribes to /coordinator/target, plans a trajectory with MoveIt2,
asks for manual confirmation, executes, then publishes status feedback.

Subscriptions:
    /coordinator/target  (geometry_msgs/PointStamped)  — 3D target in base_link frame

Publications:
    /ur5/status          (std_msgs/String)              — "done" or "error"
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


class UR5CoordinatorNode(Node):

    def __init__(self):
        super().__init__('ur5_coordinator_node')
        self.cb_group = ReentrantCallbackGroup()

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

        # ── MoveIt2 config (same as your go_to_xyz.py) ──────────────
        self.group_name = 'ur_manipulator'
        self.base_frame  = 'base_link'
        self.ee_link     = 'tool0'

        # ── Guard: ignore new targets while arm is busy ──────────────
        self.busy = False

        self.get_logger().info(
            'UR5 coordinator node ready. '
            'Waiting for targets on /coordinator/target ...')

    # ─────────────────────────────────────────────────────────────────
    def _target_callback(self, msg: PointStamped):
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
        success = self._plan_confirm_execute(x, y, z)
        self.busy = False

        # Publish feedback to coordinator
        status_msg = String()
        if success:
            status_msg.data = 'done'
            self.get_logger().info('Move complete — publishing "done"')
        else:
            status_msg.data = 'error'
            self.get_logger().warn('Move failed or cancelled — publishing "error"')

        self.status_pub.publish(status_msg)

    # ─────────────────────────────────────────────────────────────────
    def _plan_confirm_execute(self, x, y, z) -> bool:
        """
        Plan with MoveIt2, show in RViz, ask for confirmation,
        then execute. Returns True on success, False otherwise.
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
        self.get_logger().info(
            f'Planning to: ({x:.3f}, {y:.3f}, {z:.3f}) ...')

        future = self.movegroup_client.send_goal_async(plan_goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        mg_result = result_future.result().result
        err = mg_result.error_code.val

        if err != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'Planning failed — MoveIt error code: {err}')
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

        # ── Step 3: Manual confirmation ──────────────────────────────
        # NOTE: Remove this block later when running fully autonomous
        print(f'\n  Target: x={x:.3f}  y={y:.3f}  z={z:.3f}')
        ans = input('  Execute this trajectory? [y/N]: ').strip().lower()
        if ans not in ('y', 'yes'):
            self.get_logger().info('Execution cancelled by user.')
            return False

        # ── Step 4: Execute trajectory ───────────────────────────────
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = planned_traj

        self.get_logger().info('Executing trajectory...')
        exec_future = self.exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)

        exec_handle = exec_future.result()
        if not exec_handle or not exec_handle.accepted:
            self.get_logger().error('ExecuteTrajectory goal rejected')
            return False

        exec_res_future = exec_handle.get_result_async()
        rclpy.spin_until_future_complete(self, exec_res_future)

        exec_result = exec_res_future.result().result
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
        """Build a MoveGroup goal for a position target (same as go_to_xyz.py)."""
        goal = MoveGroup.Goal()
        req  = goal.request

        req.group_name                    = self.group_name
        req.allowed_planning_time         = 5.0
        req.num_planning_attempts         = 5
        req.max_velocity_scaling_factor   = 0.1
        req.max_acceleration_scaling_factor = 0.1

        # Position constraint: 5mm sphere around the target
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name       = self.ee_link

        sphere = SolidPrimitive()
        sphere.type       = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]  # 5mm tolerance

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
    node = UR5CoordinatorNode()
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
