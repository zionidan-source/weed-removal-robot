"""
Full pipeline launch file
==========================
Launches: RealSense D435i, static TF, YOLO detection, coordinator.

UR5 node is launched separately so it can run with stdin attached
(required for require_confirmation=True).  Run it with:

    ros2 run ur5_control ur5_node --ros-args --params-file \
        ~/ros2_workspaces/weed_removal_robot_ws/src/ur5_control/config/ur5_params.yaml

Usage (no arguments required):
    ros2 launch vision_pick full_pipeline.launch.py

Override examples:
    ros2 launch vision_pick full_pipeline.launch.py model_path:=/path/to/model.pt
    ros2 launch vision_pick full_pipeline.launch.py confidence_threshold:=0.6

NOTE: The static TF (base_link → camera_link) is baked-in from hand-eye calibration.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('vision_pick')

    # ── Launch arguments ────────────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/daniel/ros2_workspaces/weed_removal_robot_ws/models/Rocky7.pt',
        description='Path to the YOLOv11 .pt model file')

    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.7',
        description='YOLO minimum confidence threshold')

    process_every_n_arg = DeclareLaunchArgument(
        'process_every_n_frames',
        default_value='2',
        description='Run YOLO on every Nth frame')

    # ── 1. RealSense D435i ──────────────────────────────────────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')),
        launch_arguments={
            'config_file': os.path.join(pkg_dir, 'config', 'realsense_params.yaml'),
        }.items(),
    )

    # ── 2. Static TF: base_link → camera_link ──────────────────────────
    #   Values from hand-eye calibration (DANIILIDIS solver, 6.7 mm residual).
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link_tf',
        arguments=[
            '--x', '-0.940389',
            '--y', '0.832932',
            '--z', '0.104916',
            '--qx', '-0.010102',
            '--qy', '-0.006404',
            '--qz', '-0.415715',
            '--qw', '0.909416',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    # ── 3. YOLO Detection Node ──────────────────────────────────────────
    yolo_node = Node(
        package='vision_pick',
        executable='yolo_detection_node',
        name='yolo_detection_node',
        parameters=[
            os.path.join(pkg_dir, 'config', 'yolo_params.yaml'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'process_every_n_frames': LaunchConfiguration('process_every_n_frames'),
            },
        ],
        output='screen',
    )

    # ── 4. Coordinator Node ─────────────────────────────────────────────
    coordinator_node = Node(
        package='vision_pick',
        executable='coordinator_node',
        name='coordinator_node',
        parameters=[
            os.path.join(pkg_dir, 'config', 'coordinator_params.yaml'),
        ],
        output='screen',
    )

    return LaunchDescription([
        model_path_arg,
        confidence_arg,
        process_every_n_arg,
        realsense_launch,
        static_tf_node,
        yolo_node,
        coordinator_node,
    ])