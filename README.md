# Weed Removal Robot
Autonomous weed detection and removal system using a UR5 robotic arm, Intel RealSense D435i depth camera, and YOLOv11 detection.

## Architecture
```
┌──────────────┐     ┌──────────────┐     ┌──────────────────┐     ┌────────────┐
│  RealSense   │────▶│  YOLO Node   │────▶│  Coordinator     │────▶│  UR5 Node  │
│  D435i       │     │  (detection) │     │  (3D transform   │     │  (motion)  │
│              │────▶│              │     │   + pick queue)  │◀────│            │
│  color+depth │     │  /yolo/det.  │     │  /coord/target   │     │  /ur5/stat │
└──────────────┘     └──────────────┘     └──────────────────┘     └────────────┘
```

## Packages
| Package | Description |
|---------|-------------|
| `vision_pick` | Camera config, YOLO detection node, and coordinator node |
| `ur5_control` | UR5 arm control node (receives coordinates, executes motion) |

## ROS2 Topics
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense → YOLO | Color frames |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RealSense → Coordinator | Aligned depth |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RealSense → Coordinator | Camera intrinsics |
| `/yolo/detections` | `std_msgs/String` | YOLO → Coordinator | JSON list of detected weeds |
| `/yolo/debug_image` | `sensor_msgs/Image` | YOLO → RViz | Annotated image for debugging |
| `/coordinator/target` | `geometry_msgs/PointStamped` | Coordinator → UR5 | Next weed coordinate (base frame) |
| `/coordinator/busy` | `std_msgs/Bool` | Coordinator → Drive | Busy signal for platform control |
| `/coordinator/queue_size` | `std_msgs/Int32` | Coordinator → Monitor | Current queue length |
| `/ur5/status` | `std_msgs/String` | UR5 → Coordinator | `"done"` or `"error"` feedback |

## Prerequisites
- Ubuntu 24.04
- ROS2 Jazzy
- Intel RealSense SDK + `ros-jazzy-realsense2-camera`
- Python 3.10+
- See [INSTALL.md](INSTALL.md) for full setup instructions

## Quick Start

The pipeline and the UR5 node are launched separately. The UR5 node requires
a terminal with stdin attached so the operator can confirm each move.

```bash
# Terminal 1 — UR driver + MoveIt2
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5 \
  robot_ip:=192.168.1.113 \
  kinematics_params_file:="${HOME}/ur5_calibration.yaml" \
  controller_spawner_timeout:=60

# Terminal 2 — MoveIt2
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5 use_sim_time:=false launch_rviz:=true

# Terminal 3 — Vision pipeline (RealSense + YOLO + Coordinator)
cd ~/ros2_workspaces/weed_removal_robot_ws
source install/setup.bash
ros2 launch vision_pick full_pipeline.launch.py

# Terminal 4 — UR5 node (must have stdin — run standalone, not from launch file)
ros2 run ur5_control ur5_node --ros-args \
  --params-file ~/ros2_workspaces/weed_removal_robot_ws/src/ur5_control/config/ur5_params.yaml
```

> **Note:** Make sure the External Control URCap program is **playing** on the
> teach pendant before starting the UR driver.


## Configuration
All parameters are in YAML files under each package's `config/` directory:
- `vision_pick/config/realsense_params.yaml` — Camera resolution, filters, alignment
- `vision_pick/config/yolo_params.yaml` — Model path, confidence, frame skip
- `vision_pick/config/coordinator_params.yaml` — TF frames, reach limits, depth range
- `ur5_control/config/ur5_params.yaml` — Planning parameters, confirmation mode

## Calibration

Hand-eye calibration (eye-to-hand, AprilTag tag36h11 id 20, 64 mm) has been
performed and baked into `full_pipeline.launch.py`.

**Baked-in transform: `base_link → camera_link`**
| Parameter | Value |
|-----------|-------|
| x | -0.940389 m |
| y |  0.832932 m |
| z |  0.104916 m |
| qx | -0.010102 |
| qy | -0.006404 |
| qz | -0.415715 |
| qw |  0.909416 |

Solver: DANIILIDIS — residual 6.7 mm. Verified live against tool0 vs tag position.

To recalibrate:
```bash
python3 ~/ros2_workspaces/weed_removal_robot_ws/src/ur5_control/scripts/handeye_calibrate.py
```
Then update the `static_tf_node` arguments in `src/vision_pick/launch/full_pipeline.launch.py`.
