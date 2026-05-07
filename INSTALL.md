# Installation Guide

## 1. ROS2 Jazzy

Follow the official guide: https://docs.ros.org/en/jazzy/Installation.html

## 2. Intel RealSense SDK

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

Verify: plug in D435i and run `realsense-viewer`

## 3. ROS2 Packages

```bash
sudo apt install -y \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-vision-msgs \
  ros-jazzy-ur-robot-driver
```

## 4. MoveIt2

```bash
sudo apt install -y ros-jazzy-moveit
```

## 5. Python Dependencies

```bash
pip install ultralytics opencv-python-headless numpy
```

## 6. Clone and Build the Workspace

```bash
git clone git@github.com:zionidan-source/weed-removal-robot.ws.git ~/ros2_workspaces/weed_removal_robot.ws
cd ~/ros2_workspaces/weed_removal_robot.ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Add to your `~/.bashrc` for convenience:
```bash
echo "source ~/ros2_workspaces/weed_removal_robot.ws/install/setup.bash" >> ~/.bashrc
```

## 7. YOLO Model

The `.pt` model files are not tracked in git (they are large). Copy or download them separately and pass the path at launch time:

```bash
ros2 launch vision_pick full_pipeline.launch.py model_path:=/path/to/your/model.pt
```

## 8. UR5 Robot Connection

The UR5 requires the **ExternalControl URCap** to be installed and a program running on the pendant.

```bash
# Launch the UR5 driver (replace with your robot's IP)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=YOUR_ROBOT_IP
```

> **Note:** Only one `ur_robot_driver` instance can connect to the robot at a time.
> If you see "another program is already controlling the robot", make sure no other
> computer or terminal is running the driver. On the UR pendant: stop any running
> programs and re-launch ExternalControl before reconnecting.

## 9. Verify

```bash
# Test camera
ros2 launch realsense2_camera rs_launch.py

# Test UR5 connection (replace with your robot IP)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=YOUR_ROBOT_IP
```
