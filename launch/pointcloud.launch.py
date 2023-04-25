import os

import launch_ros
from launch_ros.actions.node import Node

from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.conditions.unless_condition import UnlessCondition
from launch.conditions.if_condition import IfCondition


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="kinect_ros2").find(
        "kinect_ros2"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/pointcloud.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument('rviz', default_value='true', choices=['false', 'true']),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                name="kinect_ros2",
                namespace="kinect",
                output = 'log',
                parameters=[
                    {"depth_frame_id": "camera_depth_optical_frame",
                     "rgb_frame_id" : "camera_rgb_optical_frame"
                             }
                    ],
                remappings=[("depth_registered/image_rect", "depth/image_raw"),
                             ("rgb/image_rect_color", "image_raw"),
                             ("rgb/camera_info", "camera_info")]
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
                condition = IfCondition(LaunchConfiguration('rviz'))
            ),
        ]
    )
