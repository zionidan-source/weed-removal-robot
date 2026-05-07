"""
Full pipeline launch file
==========================
Launches: RealSense D435i, static TF, YOLO detection, coordinator.

Usage:
    ros2 launch vision_pick full_pipeline.launch.py model_path:=/path/to/model.pt
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('vision_pick')

    # ── Launch arguments ────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to the YOLOv11 .pt model file')

    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO minimum confidence threshold')

    process_every_n_arg = DeclareLaunchArgument(
        'process_every_n_frames',
        default_value='5',
        description='Run YOLO on every Nth frame')

    # ── 1. RealSense Camera ─────────────────────────────────────
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[os.path.join(pkg_dir, 'config', 'realsense_params.yaml')],
        output='screen',
    )

    # ── 2. Static TF: camera → robot base ──────────────────────
    #
    #   *** EDIT THESE VALUES after measuring your setup! ***
    #   Translation (x, y, z) in meters.
    #   Rotation (qx, qy, qz, qw) as quaternion.
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

    # ── 3. YOLO Detection Node ──────────────────────────────────
    yolo_node = Node(
        package='vision_pick',
        executable='yolo_detection_node',
        name='yolo_detection_node',
        parameters=[
            os.path.join(pkg_dir, 'config', 'yolo_params.yaml'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration(
                    'confidence_threshold'),
                'process_every_n_frames': LaunchConfiguration(
                    'process_every_n_frames'),
            },
        ],
        output='screen',
    )

    # ── 4. Coordinator Node ─────────────────────────────────────
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
        realsense_node,
        static_tf_node,
        yolo_node,
        coordinator_node,
    ])
