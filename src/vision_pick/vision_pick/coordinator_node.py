"""
Coordinator Node
================
The "brain" of the pipeline. Takes YOLO detections + aligned depth image,
computes 3D weed positions in the robot base frame, manages a pick queue,
and sends targets one-at-a-time to the UR5 node.

Subscriptions:
    /yolo/detections                                (std_msgs/String)
    /camera/camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
    /camera/camera/color/camera_info                (sensor_msgs/CameraInfo)
    /ur5/status                                     (std_msgs/String)

Publications:
    /coordinator/target     (geometry_msgs/PointStamped)  → UR5 node
    /coordinator/busy       (std_msgs/Bool)               → drive controller
    /coordinator/queue_size (std_msgs/Int32)               → monitoring
"""

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class CoordinatorNode(Node):

    def __init__(self):
        super().__init__('coordinator_node')

        # ── Declare parameters ──────────────────────────────────────
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('max_reach_m', 0.85)
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 1.5)
        self.declare_parameter('duplicate_distance_m', 0.03)
        self.declare_parameter('depth_sample_radius_px', 5)

        # ── Read parameters ─────────────────────────────────────────
        self.robot_base_frame = self.get_parameter('robot_base_frame') \
            .get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame') \
            .get_parameter_value().string_value
        self.max_reach = self.get_parameter('max_reach_m') \
            .get_parameter_value().double_value
        self.min_depth = self.get_parameter('min_depth_m') \
            .get_parameter_value().double_value
        self.max_depth = self.get_parameter('max_depth_m') \
            .get_parameter_value().double_value
        self.duplicate_dist = self.get_parameter('duplicate_distance_m') \
            .get_parameter_value().double_value
        self.depth_sample_radius = self.get_parameter('depth_sample_radius_px') \
            .get_parameter_value().integer_value

        # ── Internal state ──────────────────────────────────────────
        self.bridge = CvBridge()
        self.camera_intrinsics = None       # filled by camera_info callback
        self.latest_depth_image = None      # filled by depth callback
        self.pick_queue = []                # list of PointStamped (base frame)
        self.arm_busy = False               # True while UR5 is executing

        # ── TF2 listener ────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers ──────────────────────────────────────────────
        self.target_pub = self.create_publisher(
            PointStamped, '/coordinator/target', 10)
        self.busy_pub = self.create_publisher(
            Bool, '/coordinator/busy', 10)
        self.queue_size_pub = self.create_publisher(
            Int32, '/coordinator/queue_size', 10)

        # ── Subscribers ─────────────────────────────────────────────
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._camera_info_callback, 10)

        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self._depth_callback, 10)

        self.create_subscription(
            String,
            '/yolo/detections',
            self._detections_callback, 10)

        self.create_subscription(
            String,
            '/ur5/status',
            self._ur5_status_callback, 10)

        # ── Periodic busy signal publisher ──────────────────────────
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'Coordinator ready. Robot frame: "{self.robot_base_frame}", '
            f'Camera frame: "{self.camera_frame}", '
            f'Max reach: {self.max_reach}m')

    # ─────────────────────────────────────────────────────────────────
    # CALLBACKS
    # ─────────────────────────────────────────────────────────────────

    def _camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics (only need to capture once)."""
        if self.camera_intrinsics is not None:
            return

        # K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5],
            'width': msg.width,
            'height': msg.height,
        }
        self.get_logger().info(
            f'Camera intrinsics received: '
            f'fx={self.camera_intrinsics["fx"]:.1f}, '
            f'fy={self.camera_intrinsics["fy"]:.1f}, '
            f'cx={self.camera_intrinsics["cx"]:.1f}, '
            f'cy={self.camera_intrinsics["cy"]:.1f}')

    def _depth_callback(self, msg: Image):
        """Store the latest aligned depth image."""
        try:
            # RealSense depth is 16-bit unsigned, in millimeters
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')

    def _detections_callback(self, msg: String):
        """Process YOLO detections → compute 3D → add to queue."""
        if self.camera_intrinsics is None:
            self.get_logger().warn(
                'Detections received but no camera intrinsics yet. Skipping.')
            return
        if self.latest_depth_image is None:
            self.get_logger().warn(
                'Detections received but no depth image yet. Skipping.')
            return

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detections JSON: {e}')
            return

        if not detections:
            return

        new_points_added = 0

        for det in detections:
            u = det['centroid_u']
            v = det['centroid_v']
            confidence = det['confidence']
            class_name = det['class_name']

            # ── Step 1: Get depth at the centroid pixel ──────────
            depth_m = self._get_depth_at_pixel(u, v)
            if depth_m is None:
                self.get_logger().debug(
                    f'No valid depth for {class_name} at ({u:.0f}, {v:.0f})')
                continue

            # ── Step 2: Pixel (u,v,depth) → 3D point in camera frame
            point_camera = self._pixel_to_3d(u, v, depth_m)

            # ── Step 3: Transform camera frame → robot base frame ──
            point_base = self._transform_to_base(point_camera)
            if point_base is None:
                continue

            # ── Step 4: Reachability safety check ────────────────
            dist = math.sqrt(
                point_base.point.x ** 2 +
                point_base.point.y ** 2 +
                point_base.point.z ** 2)
            if dist > self.max_reach:
                self.get_logger().debug(
                    f'{class_name} at ({point_base.point.x:.3f}, '
                    f'{point_base.point.y:.3f}, {point_base.point.z:.3f}) '
                    f'is {dist:.2f}m away — beyond max reach {self.max_reach}m')
                continue

            # ── Step 5: Duplicate check ──────────────────────────
            if self._is_duplicate(point_base):
                continue

            # ── Step 6: Add to queue ─────────────────────────────
            self.pick_queue.append(point_base)
            new_points_added += 1

            self.get_logger().info(
                f'Queued {class_name} (conf={confidence:.2f}) at '
                f'base frame ({point_base.point.x:.3f}, '
                f'{point_base.point.y:.3f}, {point_base.point.z:.3f})')

        if new_points_added > 0:
            # Re-sort queue by Y coordinate (smallest first)
            self.pick_queue.sort(key=lambda p: p.point.y)
            self.get_logger().info(
                f'Added {new_points_added} target(s). '
                f'Queue size: {len(self.pick_queue)}')

            # If arm is idle, send the next target immediately
            if not self.arm_busy:
                self._send_next_target()

    def _ur5_status_callback(self, msg: String):
        """Handle feedback from the UR5 node."""
        status = msg.data.strip().lower()

        if status == 'done':
            self.get_logger().info('UR5 reports: move complete.')
            self.arm_busy = False
            self._send_next_target()
        elif status == 'error':
            self.get_logger().warn(
                'UR5 reports: error. Skipping current target, '
                'sending next.')
            self.arm_busy = False
            self._send_next_target()
        else:
            self.get_logger().debug(f'UR5 status: {status}')

    # ─────────────────────────────────────────────────────────────────
    # CORE LOGIC
    # ─────────────────────────────────────────────────────────────────

    def _get_depth_at_pixel(self, u, v):
        """
        Read depth at pixel (u, v) using a small neighborhood average
        to handle noisy/missing depth pixels.
        Returns depth in meters, or None if invalid.
        """
        depth_img = self.latest_depth_image
        h, w = depth_img.shape[:2]

        u_int, v_int = int(round(u)), int(round(v))

        # Define sampling region (small square around the centroid)
        r = self.depth_sample_radius
        u_min = max(0, u_int - r)
        u_max = min(w, u_int + r + 1)
        v_min = max(0, v_int - r)
        v_max = min(h, v_int + r + 1)

        region = depth_img[v_min:v_max, u_min:u_max].astype(np.float64)

        # RealSense depth is in millimeters (uint16) — convert to meters
        region_m = region / 1000.0

        # Filter out zeros (no depth) and out-of-range values
        valid = region_m[
            (region_m > self.min_depth) & (region_m < self.max_depth)]

        if valid.size == 0:
            return None

        # Use median for robustness against outliers
        return float(np.median(valid))

    def _pixel_to_3d(self, u, v, depth_m):
        """
        Convert pixel coordinates + depth → 3D point in camera frame.
        Uses the pinhole camera model:
            X = (u - cx) * depth / fx
            Y = (v - cy) * depth / fy
            Z = depth
        """
        intr = self.camera_intrinsics

        x = (u - intr['cx']) * depth_m / intr['fx']
        y = (v - intr['cy']) * depth_m / intr['fy']
        z = depth_m

        point = PointStamped()
        point.header.frame_id = self.camera_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = z

        return point

    def _transform_to_base(self, point_camera):
        """
        Transform a PointStamped from camera frame to robot base frame
        using TF2.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                point_camera.header.frame_id,
                rclpy.time.Time(),           # latest available
                timeout=rclpy.duration.Duration(seconds=0.5))

            point_base = do_transform_point(point_camera, transform)
            point_base.header.frame_id = self.robot_base_frame
            return point_base

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF transform failed ({self.camera_frame} → '
                f'{self.robot_base_frame}): {e}')
            return None

    def _is_duplicate(self, new_point):
        """
        Check if a point is too close to an existing queue entry.
        Prevents adding the same weed multiple times across frames.
        """
        for queued in self.pick_queue:
            dx = new_point.point.x - queued.point.x
            dy = new_point.point.y - queued.point.y
            dz = new_point.point.z - queued.point.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < self.duplicate_dist:
                return True
        return False

    def _send_next_target(self):
        """Pop the first target from the queue and send it to the UR5."""
        if not self.pick_queue:
            self.get_logger().info('Queue empty — arm is idle.')
            self.arm_busy = False
            return

        target = self.pick_queue.pop(0)
        self.arm_busy = True

        self.target_pub.publish(target)
        self.get_logger().info(
            f'Sent target to UR5: ({target.point.x:.3f}, '
            f'{target.point.y:.3f}, {target.point.z:.3f}). '
            f'Remaining in queue: {len(self.pick_queue)}')

    def _publish_status(self):
        """Periodically publish busy state and queue size."""
        busy_msg = Bool()
        busy_msg.data = self.arm_busy or len(self.pick_queue) > 0
        self.busy_pub.publish(busy_msg)

        queue_msg = Int32()
        queue_msg.data = len(self.pick_queue)
        self.queue_size_pub.publish(queue_msg)


# ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
