"""
Launch file that starts the Record3D bridge node and (optionally) remaps its
topics to match the default Isaac ROS FoundationPose subscriptions.

Usage:
  ros2 launch <your_package> record3d_foundationpose.launch.py \
      mesh_file_path:=/path/to/mesh.obj \
      texture_path:=/path/to/texture.png \
      refine_model_file_path:=/path/to/refine.onnx \
      score_model_file_path:=/path/to/score.onnx
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ────────────────────────────────────────────
        DeclareLaunchArgument('device_index', default_value='0'),
        DeclareLaunchArgument('mesh_file_path', default_value=''),
        DeclareLaunchArgument('texture_path', default_value=''),
        DeclareLaunchArgument('refine_model_file_path', default_value=''),
        DeclareLaunchArgument('score_model_file_path', default_value=''),

        # ── Record3D bridge node ────────────────────────────────────────
        Node(
            package='record3d_foundationpose',       # ← your ROS 2 package name
            executable='record3d_foundationpose_node',
            name='record3d_bridge',
            output='screen',
            parameters=[{
                'device_index': LaunchConfiguration('device_index'),
                'frame_id': 'iphone_camera',
                'world_frame_id': 'world',
            }],
        ),

        # ── Isaac ROS FoundationPose node ───────────────────────────────
        Node(
            package='isaac_ros_foundationpose',
            executable='foundationpose_node',
            name='foundation_pose',
            output='screen',
            remappings=[
                # RGB
                ('image', '/iphone/color/image_raw'),
                # Depth
                ('depth', '/iphone/depth/image_raw'),
                # Camera intrinsics
                ('camera_info', '/iphone/camera_info'),
                # Segmentation mask
                ('segmentation', '/iphone/segmentation/mask'),
            ],
            parameters=[{
                'mesh_file_path':
                    LaunchConfiguration('mesh_file_path'),
                'texture_path':
                    LaunchConfiguration('texture_path'),
                'refine_model_file_path':
                    LaunchConfiguration('refine_model_file_path'),
                'score_model_file_path':
                    LaunchConfiguration('score_model_file_path'),
            }],
        ),
    ])