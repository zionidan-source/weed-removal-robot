#!/usr/bin/env python3
"""
Eye-to-hand calibration for the weed removal robot.

Setup: camera FIXED on platform, AprilTag mounted on tool0 (end-effector).
Goal:  solve base_link -> camera_color_optical_frame static transform.

Usage:
  1. Launch RealSense, apriltag_node, UR5 driver + MoveIt (so TF is populated).
  2. Run this node:  python3 handeye_calibrate.py
  3. Move the arm to a new pose, let it settle, press ENTER to capture.
     Vary ORIENTATION across poses, not just position. Aim for 12-15 captures.
  4. Type 'solve' and press ENTER to compute and print the result.
  5. Type 'quit' to exit.

Captures are saved to handeye_samples.npz so you can re-solve later.
"""

import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

# --- frames: edit if yours differ ---
BASE_FRAME = "base_link"
EE_FRAME = "tool0"
CAMERA_FRAME = "camera_color_optical_frame"
TAG_FRAME = "tag36h11:20"   # apriltag_ros default naming: family:id
SAMPLE_FILE = "handeye_samples.npz"
MAX_TAG_AGE_S = 0.5         # reject captures whose tag detection is older than this


def quat_to_R(x, y, z, w):
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def tf_to_Rt(t):
    tr = t.transform.translation
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    tvec = np.array([tr.x, tr.y, tr.z])
    return R, tvec


class HandEye(Node):
    def __init__(self):
        super().__init__("handeye_calibrate")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        # gripper(tool0)->base  and  target(tag)->camera  per OpenCV convention
        self.R_g2b, self.t_g2b = [], []
        self.R_t2c, self.t_t2c = [], []

    def lookup(self, parent, child):
        return self.buf.lookup_transform(
            parent, child, rclpy.time.Time(),
            timeout=Duration(seconds=2.0))

    def capture(self):
        try:
            # base -> tool0  (we want tool0 expressed in base = gripper2base)
            t_b2e = self.lookup(BASE_FRAME, EE_FRAME)
            # camera -> tag   (tag expressed in camera = target2cam)
            t_c2t = self.lookup(CAMERA_FRAME, TAG_FRAME)
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return False

        # Reject stale tag detections. With a slow detector, the tag TF can be
        # several seconds old -> it would pair the CURRENT arm pose with a tag
        # pose from a PREVIOUS arm position, corrupting the calibration.
        now = self.get_clock().now()
        tag_stamp = rclpy.time.Time.from_msg(t_c2t.header.stamp)
        age = (now - tag_stamp).nanoseconds / 1e9
        if age > MAX_TAG_AGE_S:
            self.get_logger().error(
                f"REJECTED: tag detection is {age:.2f}s old (max {MAX_TAG_AGE_S}s). "
                f"Wait for a fresh detection, then capture again.")
            return False

        R_b2e, t_b2e_v = tf_to_Rt(t_b2e)
        R_c2t, t_c2t_v = tf_to_Rt(t_c2t)
        self.R_g2b.append(R_b2e); self.t_g2b.append(t_b2e_v)
        self.R_t2c.append(R_c2t); self.t_t2c.append(t_c2t_v)
        n = len(self.R_g2b)
        self.get_logger().info(
            f"Captured pose {n}. tool0 pos={t_b2e_v.round(3)} | tag age={age:.2f}s")
        self.save()
        return True

    def save(self):
        np.savez(SAMPLE_FILE,
                 R_g2b=np.array(self.R_g2b), t_g2b=np.array(self.t_g2b),
                 R_t2c=np.array(self.R_t2c), t_t2c=np.array(self.t_t2c))

    def solve(self):
        n = len(self.R_g2b)
        if n < 3:
            self.get_logger().error(f"Need >=3 poses (have {n}). Aim for 12+.")
            return

        # Diagnostic: how much did the tag ORIENTATION vary across poses?
        # Hand-eye needs wide rotational spread or the solution is unstable.
        # Compute pairwise relative rotation angles between tag poses.
        angles = []
        for i in range(n):
            for j in range(i + 1, n):
                R_rel = self.R_t2c[i].T @ self.R_t2c[j]
                cos = (np.trace(R_rel) - 1.0) / 2.0
                cos = max(-1.0, min(1.0, cos))
                angles.append(np.degrees(np.arccos(cos)))
        max_ang = max(angles) if angles else 0.0
        print(f"\n[diagnostic] max pairwise tag-orientation change: {max_ang:.1f} deg")
        if max_ang < 30.0:
            print("[diagnostic] WARNING: orientation spread is small (<30 deg). "
                  "Result will likely be unstable. Re-capture with the tag tilted "
                  "to MUCH more varied angles.")

        # Eye-to-hand (VALIDATED convention):
        #  - feed OpenCV the INVERTED robot pose (base2gripper)
        #  - read the output DIRECTLY as base->camera (no extra inversion)
        # We try all methods and keep the one with the lowest residual, where
        # residual = how constant the implied tool0->tag offset is across poses
        # (it MUST be constant, since the tag is rigidly fixed to the flange).
        R_b2g, t_b2g = [], []
        for R, t in zip(self.R_g2b, self.t_g2b):
            R_inv = R.T
            R_b2g.append(R_inv)
            t_b2g.append((-R_inv @ t).reshape(3, 1))
        t2c_cols = [t.reshape(3, 1) for t in self.t_t2c]

        methods = {
            "TSAI": cv2.CALIB_HAND_EYE_TSAI,
            "PARK": cv2.CALIB_HAND_EYE_PARK,
            "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
            "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
            "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }

        def residual(R_bc, t_bc):
            ys = []
            for i in range(n):
                Rb2t = self.R_g2b[i].T
                tb2t = -Rb2t @ self.t_g2b[i]
                R1 = Rb2t @ R_bc
                t1 = Rb2t @ t_bc.flatten() + tb2t
                t2 = R1 @ self.t_t2c[i] + t1
                ys.append(t2)
            return np.array(ys).std(axis=0).sum()

        best = None
        for name, m in methods.items():
            try:
                R_bc, t_bc = cv2.calibrateHandEye(R_b2g, t_b2g, self.R_t2c, t2c_cols, method=m)
            except Exception:
                continue
            res = residual(R_bc, t_bc.flatten())
            if best is None or res < best[0]:
                best = (res, name, R_bc, t_bc.flatten())

        res, name, R_base2cam, t_base2cam = best
        print(f"[solver] best method: {name}, residual: {res*1000:.1f} mm "
              f"(want < ~15 mm)")
        self.report(R_base2cam, t_base2cam, n)

    def report(self, R, t, n):
        # rotation matrix -> quaternion
        tr = np.trace(R)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * s
            qx = (R[2,1] - R[1,2]) / s
            qy = (R[0,2] - R[2,0]) / s
            qz = (R[1,0] - R[0,1]) / s
        else:
            i = np.argmax([R[0,0], R[1,1], R[2,2]])
            if i == 0:
                s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
                qw = (R[2,1] - R[1,2]) / s; qx = 0.25 * s
                qy = (R[0,1] + R[1,0]) / s; qz = (R[0,2] + R[2,0]) / s
            elif i == 1:
                s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
                qw = (R[0,2] - R[2,0]) / s; qx = (R[0,1] + R[1,0]) / s
                qy = 0.25 * s; qz = (R[1,2] + R[2,1]) / s
            else:
                s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
                qw = (R[1,0] - R[0,1]) / s; qx = (R[0,2] + R[2,0]) / s
                qy = (R[1,2] + R[2,1]) / s; qz = 0.25 * s
        print("\n" + "="*60)
        print(f"RESULT  (base_link -> {CAMERA_FRAME})   from {n} poses")
        print("="*60)
        print(f"translation (m): x={t[0]:.4f}  y={t[1]:.4f}  z={t[2]:.4f}")
        print(f"quaternion:      qx={qx:.5f} qy={qy:.5f} qz={qz:.5f} qw={qw:.5f}")
        print("\nStatic TF command:")
        print(f"ros2 run tf2_ros static_transform_publisher \\")
        print(f"  --x {t[0]:.4f} --y {t[1]:.4f} --z {t[2]:.4f} \\")
        print(f"  --qx {qx:.5f} --qy {qy:.5f} --qz {qz:.5f} --qw {qw:.5f} \\")
        print(f"  --frame-id base_link --child-frame-id {CAMERA_FRAME}")
        print("="*60 + "\n")


def main():
    rclpy.init()
    node = HandEye()

    # Spin in a background thread so the TF listener fills its buffer
    # continuously, even while the main thread is blocked on input().
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Give the buffer a moment to populate before the first prompt.
    import time
    time.sleep(1.0)

    print("Hand-eye calibration. Commands: [ENTER]=capture, 'solve', 'quit'")
    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "quit":
                break
            elif cmd == "solve":
                node.solve()
            else:
                node.capture()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
