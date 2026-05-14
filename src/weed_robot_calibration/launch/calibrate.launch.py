"""
Hand-eye calibration launch file
==================================
Brings up everything needed for eye-on-base calibration:
  UR5 driver → MoveIt2 → RealSense → ArUco detection → easy_handeye2 GUI

Setup (one-time):
  1. Clone external dependencies into this workspace and rebuild:
       cd ~/ros2_workspaces/weed_removal_robot_ws/src
       git clone -b jazzy https://github.com/ycheng517/easy_handeye2.git
       git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git
       cd .. && colcon build

  2. Print an ArUco marker (DICT_5X5_250, ID 1, 7 cm).
     Adjust config/aruco_parameters.yaml → marker_size to match exactly.

  3. Mount the marker flat on the UR5 end-effector (tool0 flange).

Usage:
  ros2 launch weed_robot_calibration calibrate.launch.py robot_ip:=<ip>

  Optional overrides:
    marker_size:=0.05     (if you printed a 5 cm marker)
    ur_type:=ur5          (default)

Calibration workflow:
  - RViz opens with robot + camera feed
  - Move arm to 5-6 diverse poses where the marker is clearly visible
  - Use the easy_handeye2 GUI: click "Take sample" at each pose
  - Click "Compute" then "Save" — result is broadcast on /tf_static
  - Copy the saved transform values into full_pipeline.launch.py static TF
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('weed_robot_calibration')

    # ── Launch arguments ────────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        description='IP address of the UR5 robot')

    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5',
        description='UR robot model (ur5, ur10, etc.)')

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.07',
        description='ArUco marker side length in metres')

    # ── 1. RealSense D435i ──────────────────────────────────────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')),
        launch_arguments={
            'rgb_camera.color_profile': '1280x720x30',
            'align_depth.enable': 'false',
        }.items(),
    )

    # ── 2. UR5 driver ───────────────────────────────────────────────────
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_robot_driver'),
                'launch', 'ur_control.launch.py')),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'launch_rviz': 'false',
        }.items(),
    )

    # ── 3. MoveIt2 ──────────────────────────────────────────────────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_moveit_config'),
                'launch', 'ur_moveit.launch.py')),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': 'true',
        }.items(),
    )

    # ── 4. ArUco detection ──────────────────────────────────────────────
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        parameters=[
            os.path.join(pkg_dir, 'config', 'aruco_parameters.yaml'),
            {'marker_size': LaunchConfiguration('marker_size')},
        ],
        output='screen',
    )

    # ── 5. easy_handeye2 calibration ────────────────────────────────────
    handeye_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('easy_handeye2'),
                'launch', 'calibrate.launch.py')),
        launch_arguments={
            'name': 'weed_robot_calibration',
            'calibration_type': 'eye_on_base',
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'tool0',
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'aruco_marker_frame',
        }.items(),
    )

    return LaunchDescription([
        robot_ip_arg,
        ur_type_arg,
        marker_size_arg,
        realsense_launch,
        ur_driver_launch,
        moveit_launch,
        aruco_node,
        handeye_launch,
    ])
