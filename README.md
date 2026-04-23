# Weed Removal Robot

Autonomous weed detection and removal system using a UR5 robotic arm, Intel RealSense D435i depth camera, and YOLOv11 segmentation.

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
| `/ur5/status` | `std_msgs/String` | UR5 → Coordinator | "done" or "error" feedback |

## Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy
- Intel RealSense SDK + `ros-jazzy-realsense2-camera`
- Python 3.10+
- See [INSTALL.md](INSTALL.md) for full setup instructions

## Quick Start

```bash
# 1. Clone and build
cd ~/weed_removal_robot
colcon build
source install/setup.bash

# 2. Launch everything
ros2 launch vision_pick full_pipeline.launch.py model_path:=/path/to/your/model.pt
```

## Configuration

All parameters are in YAML files under each package's `config/` directory:

- `vision_pick/config/realsense_params.yaml` — Camera resolution, filters, alignment
- `vision_pick/config/yolo_params.yaml` — Model path, confidence, frame skip
- `vision_pick/config/coordinator_params.yaml` — TF frames, reach limits, depth range
- `ur5_control/config/ur5_params.yaml` — Robot IP, speed, acceleration

## Calibration

You must set the camera-to-robot-base transform in:
`vision_pick/launch/full_pipeline.launch.py`

To find your UR5's TF frames:
```bash
ros2 run tf2_tools view_frames
```
