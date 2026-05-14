"""
Full pipeline launch file
==========================
Launches: RealSense D435i, static TF, YOLO detection, coordinator, UR5 node.

Usage (no arguments required):
    ros2 launch vision_pick full_pipeline.launch.py

Override examples:
    ros2 launch vision_pick full_pipeline.launch.py model_path:=/path/to/model.pt
    ros2 launch vision_pick full_pipeline.launch.py confidence_threshold:=0.6

NOTE: ur5_node is included here with require_confirmation:=false (set in
ur5_params.yaml), so it auto-executes trajectories after RViz preview.
To get per-move manual approval, set require_confirmation: true in
ur5_params.yaml and run ur5_node in a separate terminal with stdin attached:
    ros2 run ur5_control ur5_node --ros-args --params-file \\
        ~/ros2_workspaces/weed_removal_robot_ws/src/ur5_control/config/ur5_params.yaml

NOTE: The static TF (base_link → camera_link) uses placeholder values.
Replace after running:  ros2 launch weed_robot_calibration calibrate.launch.py
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
    ur5_pkg_dir = get_package_share_directory('ur5_control')

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
        default_value='4',
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
    #
    #   *** PLACEHOLDER — replace x/y/z/qx/qy/qz/qw after calibration! ***
    #
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_tf',
        arguments=[
            # x     y     z       (translation in meters)
            '0.0', '0.2', '0.0',
            # qx    qy    qz    qw   (rotation quaternion)
            '0.0', '0.0', '0.0', '1.0',
            # parent_frame   child_frame
            'base_link', 'camera_link',
        ],
        output='screen',
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

    # ── 5. UR5 Node ─────────────────────────────────────────────────────
    ur5_node = Node(
        package='ur5_control',
        executable='ur5_node',
        name='ur5_node',
        parameters=[
            os.path.join(ur5_pkg_dir, 'config', 'ur5_params.yaml'),
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
        ur5_node,
    ])
