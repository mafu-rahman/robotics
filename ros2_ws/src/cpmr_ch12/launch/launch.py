import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    return LaunchDescription([
        Node(
             package='cpmr_ch12',
             executable='opencv_camera',
             name='opencv_camera',
             output='screen',
             ),
        Node(
             package='cpmr_ch12',
             executable='yolo_pose',
             name='yolo_pose',
             output='screen',
             ),
    ])

