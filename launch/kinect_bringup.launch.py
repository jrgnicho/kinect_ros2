import os
from pathlib import Path
import launch_ros
from launch_ros.actions.node import Node
from launch.actions import OpaqueFunction
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    kinect_ros2_pkg_share = get_package_share_directory('kinect_ros2')
    kinect_description_pkg_share = get_package_share_directory('kinect_description')
    kinect_urdf_model_path = LaunchConfiguration('kinect_urdf').perform(context)
    robot_description = ParameterValue(Command(['xacro ', kinect_urdf_model_path]),value_type=str)
    load_urdf = LaunchConfiguration('load_urdf').perform(context).lower() == 'true'
    
    launch_actions = []
    if(load_urdf):    
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )
        
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        )  
        launch_actions.append(robot_state_publisher_node)
        launch_actions.append(joint_state_publisher_node)
    
    
    kinect_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( str(Path(kinect_ros2_pkg_share)/'launch'/'pointcloud.launch.py')),
        launch_arguments = {'rviz' : LaunchConfiguration('rviz')}.items())    
    launch_actions.append(kinect_driver_launch)
    
    
    return launch_actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value= 'true', choices=['false', 'true']),
        DeclareLaunchArgument('load_urdf', default_value='true', choices=['false', 'true']),
        DeclareLaunchArgument('kinect_urdf', 
                              default_value= str(Path(get_package_share_directory('kinect_description'))/'urdf'/'kinect.urdf.xacro')),
        OpaqueFunction(function = launch_setup)
        ])

