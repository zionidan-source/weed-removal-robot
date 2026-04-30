"""
YOLO Weed Detection Node
=========================
Subscribes to the RealSense color image, runs YOLOv11 segmentation,
and publishes a sorted list of detected weeds (smallest Y first).

Published topic:
    /yolo/detections  (std_msgs/String)  — JSON list of detections

Each detection contains:
    - class_name: str          (weed class label)
    - class_id: int            (numeric class ID)
    - confidence: float        (0.0 - 1.0)
    - centroid_u: float        (mask centroid, pixel X)
    - centroid_v: float        (mask centroid, pixel Y)
    - mask_points: list        (contour points of the segmentation mask)

The list is sorted by centroid_v ascending (smallest Y / top of image first),
so the weed closest to leaving the robot's reach is first.
"""

import json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetectionNode(Node):

    def __init__(self):
        super().__init__('yolo_detection_node')

        # ── Declare parameters ──────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('process_every_n_frames', 5)
        self.declare_parameter('input_image_topic',
                               '/camera/camera/color/image_raw')

        # ── Read parameters ─────────────────────────────────────────
        model_path = self.get_parameter('model_path') \
                         .get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold') \
                                  .get_parameter_value().double_value
        self.process_every_n = self.get_parameter('process_every_n_frames') \
                                   .get_parameter_value().integer_value
        input_topic = self.get_parameter('input_image_topic') \
                          .get_parameter_value().string_value

        # ── Validate model path ─────────────────────────────────────
        if not model_path:
            self.get_logger().fatal(
                'Parameter "model_path" is required. '
                'Set it to the path of your .pt model file.')
            raise SystemExit(1)

        # ── Load YOLO model (CPU) ───────────────────────────────────
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        self.model.to('cpu')
        self.get_logger().info('YOLO model loaded successfully on CPU.')

        # ── CV bridge ───────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Frame counter (for skip logic) ──────────────────────────
        self.frame_count = 0

        # ── Publisher: detection results as JSON ────────────────────
        self.detections_pub = self.create_publisher(
            String, '/yolo/detections', 10)

        # ── Publisher: annotated image for debugging/visualization ──
        self.debug_image_pub = self.create_publisher(
            Image, '/yolo/debug_image', 10)

        # ── Subscriber: color image from RealSense ──────────────────
        self.image_sub = self.create_subscription(
            Image, input_topic, self._image_callback, 10)

        self.get_logger().info(
            f'YOLO node ready. Processing every {self.process_every_n} frames '
            f'with confidence >= {self.conf_threshold}')

    # ─────────────────────────────────────────────────────────────────
    def _image_callback(self, msg: Image):
        """Called on every incoming color frame."""

        # Skip frames to save CPU
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        # Convert ROS Image → OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge conversion failed: {e}')
            return

        # Run YOLOv11 segmentation inference
        results = self.model(
            cv_image,
            conf=self.conf_threshold,
            verbose=False
        )

        detections = self._extract_detections(results, cv_image)

        # Sort by centroid_v ascending (smallest Y first = leaves reach first)
        detections.sort(key=lambda d: d['centroid_v'])

        # Publish detections as JSON string
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detections_pub.publish(det_msg)

        if len(detections) > 0:
            self.get_logger().info(
                f'Detected {len(detections)} weed(s). '
                f'First target at pixel ({detections[0]["centroid_u"]:.0f}, '
                f'{detections[0]["centroid_v"]:.0f})')

        # Publish debug image with masks drawn
        self._publish_debug_image(cv_image, results, detections, msg.header)

#______________________________________________________________
    def _extract_detections(self, results, cv_image):
        """Parse YOLO results into a list of detection dicts."""
        detections = []
        result = results[0]

        if result.boxes is None or len(result.boxes) == 0:
            return detections

        # Classes to exclude (crops we don't want to remove)
        excluded_classes = {'Maize'}

        boxes = result.boxes
        has_masks = result.masks is not None

        for i in range(len(boxes)):
            conf = float(boxes.conf[i])
            class_id = int(boxes.cls[i])
            class_name = self.model.names[class_id]

            # Skip crops (e.g. Maize)
            if class_name in excluded_classes:
                continue

            # Get bounding box coordinates: xyxy = [x1, y1, x2, y2]
            xyxy = boxes.xyxy[i].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            centroid_u = float((x1 + x2) / 2.0)
            centroid_v = float((y1 + y2) / 2.0)

            # If model has masks, refine centroid using mask moments
            mask_points = []
            if has_masks and i < len(result.masks.data):
                mask = result.masks.data[i].cpu().numpy()
                if mask.shape[:2] != cv_image.shape[:2]:
                    mask = cv2.resize(
                        mask, (cv_image.shape[1], cv_image.shape[0]),
                        interpolation=cv2.INTER_NEAREST)
                binary_mask = (mask > 0.5).astype(np.uint8)
                moments = cv2.moments(binary_mask)
                if moments['m00'] > 0:
                    centroid_u = moments['m10'] / moments['m00']
                    centroid_v = moments['m01'] / moments['m00']
                    contours, _ = cv2.findContours(
                        binary_mask, cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        largest = max(contours, key=cv2.contourArea)
                        epsilon = 0.02 * cv2.arcLength(largest, True)
                        approx = cv2.approxPolyDP(largest, epsilon, True)
                        mask_points = approx.reshape(-1, 2).tolist()

            detections.append({
                'class_name': class_name,
                'class_id': class_id,
                'confidence': round(conf, 3),
                'centroid_u': round(centroid_u, 2),
                'centroid_v': round(centroid_v, 2),
                'mask_points': mask_points,
            })

        return detections

    # ─────────────────────────────────────────────────────────────────
    def _publish_debug_image(self, cv_image, results, detections, header):
        """Publish an annotated image for RViz / debugging."""
        if self.debug_image_pub.get_subscription_count() == 0:
            return  # no one listening, skip the work

        debug_img = cv_image.copy()

        # Draw YOLO's built-in annotated result
        annotated = results[0].plot()
        debug_img = annotated

        # Draw centroid markers and Y-priority numbers
        for idx, det in enumerate(detections):
            u, v = int(det['centroid_u']), int(det['centroid_v'])
            # Green circle at centroid
            cv2.circle(debug_img, (u, v), 8, (0, 255, 0), -1)
            # Priority number (1 = first to be picked)
            cv2.putText(debug_img, str(idx + 1), (u + 12, v + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish debug image: {e}')


# ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
