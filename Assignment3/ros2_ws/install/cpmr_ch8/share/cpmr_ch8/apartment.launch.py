import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Path to the world file
    world_file_path = os.path.expanduser("~/Desktop/robotics/Assignment3/apartment.sdf")
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                                       'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file_path}.items()
        ),
    ])
