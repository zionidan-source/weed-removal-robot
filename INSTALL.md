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

## 4. Python Dependencies

```bash
pip install ultralytics opencv-python-headless numpy
```

## 5. Build the Workspace

```bash
cd ~/weed_removal_robot
colcon build
source install/setup.bash
```

Add to your `~/.bashrc` for convenience:
```bash
echo "source ~/weed_removal_robot/install/setup.bash" >> ~/.bashrc
```

## 6. Verify

```bash
# Test camera
ros2 launch realsense2_camera rs_launch.py

# Test UR5 connection (replace with your robot IP)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=YOUR_ROBOT_IP
```
