#!/usr/bin/env python3
"""
UR5 Node
========
Bridges the vision coordinator and MoveIt2.
Subscribes to /coordinator/target, plans a trajectory with MoveIt2,
optionally asks for manual confirmation (with replan option), executes,
then publishes status.

Subscriptions:
    /coordinator/target     (geometry_msgs/PointStamped)  — 3D target in base_link frame
    /coordinator/candidates (std_msgs/String)             — JSON candidate list from coordinator
    /ur5/stop               (std_msgs/Empty)              — abort/cancel the current move

Publications:
    /ur5/status             (std_msgs/String)              — "done", "declined", "stopped", or "error"
    /coordinator/select_target (std_msgs/Int32)           — chosen candidate id

Parameters (see ur5_params.yaml):
    require_confirmation            (bool,  default True)  — prompt before executing
    interactive_target_selection    (bool,  default True)  — prompt to pick a target from candidates
    allowed_planning_time           (float, default 5.0)
    num_planning_attempts           (int,   default 5)
    max_velocity_scaling_factor     (float, default 0.1)
    max_acceleration_scaling_factor (float, default 0.1)
    enable_top_down_orientation     (bool,  default True)  — constrain gripper to point straight down
    approach_orientation_x/y/z/w   (float, default 1/0/0/0) — target quaternion in base_link frame
    orientation_tolerance_tilt      (float, default 0.1 rad) — tolerance for X/Y axis (tilt)
    orientation_tolerance_spin      (float, default 3.0 rad) — tolerance for Z axis (free spin)
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    MoveItErrorCodes, DisplayTrajectory
)
from geometry_msgs.msg import PointStamped, PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String, Int32, Empty


class UR5Node(Node):

    def __init__(self):
        super().__init__('ur5_node')
        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('require_confirmation', True)
        self.declare_parameter('interactive_target_selection', True)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('max_velocity_scaling_factor', 0.1)
        self.declare_parameter('max_acceleration_scaling_factor', 0.1)
        self.declare_parameter('enable_top_down_orientation', True)
        self.declare_parameter('approach_orientation_x', 1.0)
        self.declare_parameter('approach_orientation_y', 0.0)
        self.declare_parameter('approach_orientation_z', 0.0)
        self.declare_parameter('approach_orientation_w', 0.0)
        self.declare_parameter('orientation_tolerance_tilt', 0.1)
        self.declare_parameter('orientation_tolerance_spin', 3.0)

        self.require_confirmation = self.get_parameter('require_confirmation') \
            .get_parameter_value().bool_value
        self.interactive_target_selection = self.get_parameter('interactive_target_selection') \
            .get_parameter_value().bool_value
        self.allowed_planning_time = self.get_parameter('allowed_planning_time') \
            .get_parameter_value().double_value
        self.num_planning_attempts = self.get_parameter('num_planning_attempts') \
            .get_parameter_value().integer_value
        self.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling_factor') \
            .get_parameter_value().double_value
        self.max_acceleration_scaling_factor = self.get_parameter('max_acceleration_scaling_factor') \
            .get_parameter_value().double_value
        self.enable_top_down_orientation = self.get_parameter('enable_top_down_orientation') \
            .get_parameter_value().bool_value
        self.approach_orientation_x = self.get_parameter('approach_orientation_x') \
            .get_parameter_value().double_value
        self.approach_orientation_y = self.get_parameter('approach_orientation_y') \
            .get_parameter_value().double_value
        self.approach_orientation_z = self.get_parameter('approach_orientation_z') \
            .get_parameter_value().double_value
        self.approach_orientation_w = self.get_parameter('approach_orientation_w') \
            .get_parameter_value().double_value
        self.orientation_tolerance_tilt = self.get_parameter('orientation_tolerance_tilt') \
            .get_parameter_value().double_value
        self.orientation_tolerance_spin = self.get_parameter('orientation_tolerance_spin') \
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

        # ── Target selection publisher ───────────────────────────────
        self.select_target_pub = self.create_publisher(
            Int32, '/coordinator/select_target', 10)

        # ── Subscriber: target from coordinator ──────────────────────
        self.create_subscription(
            PointStamped,
            '/coordinator/target',
            self._target_callback, 10,
            callback_group=self.cb_group)

        # ── Subscriber: candidate list from coordinator ──────────────
        self.create_subscription(
            String,
            '/coordinator/candidates',
            self._candidates_callback, 10,
            callback_group=self.cb_group)

        # ── Subscriber: emergency stop / cancel current move ─────────
        self.create_subscription(
            Empty,
            '/ur5/stop',
            self._stop_callback, 10,
            callback_group=self.cb_group)

        # ── MoveIt2 config ───────────────────────────────────────────
        self.group_name = 'ur_manipulator'
        self.base_frame = 'base_link'
        self.ee_link    = 'tool0'

        # ── State ────────────────────────────────────────────────────
        self.busy = False
        self.awaiting_selection = False
        self._selection_timeout_timer = None
        self._exec_goal_handle = None       # active ExecuteTrajectory goal handle
        self._stop_requested = False        # set True when /ur5/stop is received

        self.get_logger().info(
            f'UR5 node ready '
            f'(require_confirmation={self.require_confirmation}, '
            f'interactive_target_selection={self.interactive_target_selection}, '
            f'enable_top_down_orientation={self.enable_top_down_orientation}). '
            'Waiting for targets on /coordinator/target ...')

    # ─────────────────────────────────────────────────────────────────
    async def _target_callback(self, msg: PointStamped):
        """Called when coordinator dispatches a weed coordinate."""

        if self.busy:
            self.get_logger().warn(
                'Received new target while arm is busy — ignoring. '
                'Coordinator should wait for "done" before sending next.')
            return

        # Cancel stale selection timeout if one was running
        self.awaiting_selection = False
        if self._selection_timeout_timer is not None:
            self._selection_timeout_timer.cancel()
            self._selection_timeout_timer = None

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        self.get_logger().info(
            f'Target received: ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'in frame "{msg.header.frame_id}"')

        self.busy = True
        self._stop_requested = False
        result = await self._plan_confirm_execute(x, y, z)
        self.busy = False

        status_msg = String()
        if result is True:
            status_msg.data = 'done'
            self.get_logger().info('Move complete — publishing "done"')
        elif result == 'declined':
            status_msg.data = 'declined'
            self.get_logger().info('Execution declined — publishing "declined"')
        elif result == 'stopped':
            status_msg.data = 'stopped'
            self.get_logger().warn('Execution stopped by user — publishing "stopped"')
        else:
            status_msg.data = 'error'
            self.get_logger().warn('Move failed — publishing "error"')

        self.status_pub.publish(status_msg)

    # ─────────────────────────────────────────────────────────────────
    def _candidates_callback(self, msg: String):
        """
        Called whenever the coordinator publishes the current candidate queue.
        If interactive selection is enabled and the arm is free, prompt the
        operator to pick a target id.
        """
        if self.busy or self.awaiting_selection or not self.interactive_target_selection:
            return

        try:
            candidates = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse candidates JSON: {e}')
            return

        if not candidates:
            return

        self.awaiting_selection = True

        print('\nCandidate targets:')
        for c in candidates:
            print(f"  id={c['id']:>3}  x={c['x']:.3f}  y={c['y']:.3f}  z={c['z']:.3f}")

        ans = input("Select target id to plan (or 'q' to wait): ").strip().lower()

        if ans in ('q', ''):
            self.get_logger().info('No target selected — waiting.')
            self.awaiting_selection = False
            return

        try:
            chosen_id = int(ans)
        except ValueError:
            self.get_logger().warn(f'Invalid input "{ans}" — ignoring.')
            self.awaiting_selection = False
            return

        valid_ids = {c['id'] for c in candidates}
        if chosen_id not in valid_ids:
            self.get_logger().warn(
                f'id={chosen_id} is not a current candidate — ignoring.')
            self.awaiting_selection = False
            return

        sel_msg = Int32()
        sel_msg.data = chosen_id
        self.select_target_pub.publish(sel_msg)
        self.get_logger().info(
            f'Requested target id={chosen_id} — awaiting dispatch...')

        # Start a 2s fallback timer in case coordinator can't find the id
        # (stale selection) so we don't stay stuck in awaiting_selection forever.
        self._selection_timeout_timer = self.create_timer(
            2.0, self._selection_timeout_callback)

    # ─────────────────────────────────────────────────────────────────
    def _selection_timeout_callback(self):
        """Clear awaiting_selection if the coordinator never dispatched our pick."""
        self._selection_timeout_timer.cancel()
        self._selection_timeout_timer = None
        if self.awaiting_selection:
            self.get_logger().warn(
                'Selected target not dispatched within 2s (stale id?) — '
                'will re-prompt on next candidate update.')
            self.awaiting_selection = False

    # ─────────────────────────────────────────────────────────────────
    def _stop_callback(self, msg: Empty):
        """
        Abort the current motion. Cancels the active ExecuteTrajectory goal,
        which MoveIt forwards to the controller to halt the arm.
        """
        self._stop_requested = True
        if self._exec_goal_handle is not None:
            self.get_logger().warn(
                'STOP received — cancelling trajectory execution.')
            self._exec_goal_handle.cancel_goal_async()
        else:
            self.get_logger().info(
                'STOP received — no trajectory is currently executing. '
                'A pending/next execution will be skipped.')

    # ─────────────────────────────────────────────────────────────────
    async def _plan_confirm_execute(self, x, y, z):
        """
        Plan with MoveIt2, show in RViz, optionally confirm (with replan
        option), then execute.
        Returns True on success, 'declined' if user skipped, False on failure.
        """

        # ── Wait for MoveIt2 servers ─────────────────────────────────
        self.get_logger().info('Waiting for MoveIt2 servers...')
        if not self.movegroup_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available!')
            return False
        if not self.exec_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ExecuteTrajectory action server not available!')
            return False

        planned_traj = None
        start_state = None

        while True:
            # ── Step 1: Plan only ────────────────────────────────────
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

            # ── Step 2: Publish to RViz ──────────────────────────────
            disp = DisplayTrajectory()
            disp.trajectory_start = start_state
            disp.trajectory.append(planned_traj)
            self.display_pub.publish(disp)
            self.get_logger().info(
                'Plan found. Check RViz (/display_planned_path) to preview.')

            # ── Step 3: Confirmation (with replan option) ────────────
            if not self.require_confirmation:
                break

            self.get_logger().info(
                f'Target: x={x:.3f}  y={y:.3f}  z={z:.3f}')
            ans = input(
                '  [y]es execute / [n]o skip / [r]eplan: ').strip().lower()

            if ans in ('y', 'yes'):
                break
            elif ans in ('r', 'replan'):
                self.get_logger().info('Replanning...')
                continue
            else:
                self.get_logger().info('Execution cancelled by user.')
                return 'declined'

        # ── Step 4: Execute trajectory ───────────────────────────────
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = planned_traj

        # If a stop arrived between confirmation and now, abort before moving.
        if self._stop_requested:
            self.get_logger().warn('Stop requested before execution — not moving.')
            return 'stopped'

        self.get_logger().info(
            'Executing trajectory... (publish to /ur5/stop to abort)')
        exec_handle = await self.exec_client.send_goal_async(exec_goal)

        if not exec_handle or not exec_handle.accepted:
            self.get_logger().error('ExecuteTrajectory goal rejected')
            return False

        # Expose the handle so _stop_callback can cancel it mid-motion.
        self._exec_goal_handle = exec_handle
        try:
            exec_res_response = await exec_handle.get_result_async()
        finally:
            self._exec_goal_handle = None

        if self._stop_requested:
            self.get_logger().warn('Trajectory execution cancelled (stopped).')
            return 'stopped'

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
        """Build a MoveGroup goal for a position (+ optional orientation) target."""
        goal = MoveGroup.Goal()
        req  = goal.request

        req.group_name                      = self.group_name
        req.allowed_planning_time           = self.allowed_planning_time
        req.num_planning_attempts           = self.num_planning_attempts
        req.max_velocity_scaling_factor     = self.max_velocity_scaling_factor
        req.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor

        # ── Position constraint: 5 mm sphere around the target ───────
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

        # ── Orientation constraint: top-down approach ─────────────────
        # Goal quaternion (1,0,0,0) = base_link rotated 180° about X axis
        # → tool0 Z-axis points straight down toward the ground.
        # Tune approach_orientation_* in ur5_params.yaml if needed.
        if self.enable_top_down_orientation:
            oc = OrientationConstraint()
            oc.header.frame_id = self.base_frame
            oc.link_name       = self.ee_link
            oc.orientation.x   = self.approach_orientation_x
            oc.orientation.y   = self.approach_orientation_y
            oc.orientation.z   = self.approach_orientation_z
            oc.orientation.w   = self.approach_orientation_w
            oc.absolute_x_axis_tolerance = self.orientation_tolerance_tilt
            oc.absolute_y_axis_tolerance = self.orientation_tolerance_tilt
            oc.absolute_z_axis_tolerance = self.orientation_tolerance_spin
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)

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
