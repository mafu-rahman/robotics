import os
import json
import sys
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    nchairs = 5
    for arg in sys.argv: # there must be a better way...
        if arg.startswith('nchairs:='):
           print(arg.split('chairs:=', 1)[1])
           nchairs = int(arg.split('chairs:=', 1)[1])
        elif ':=' in arg:
           print(f"Unknown argument in {arg}")
           sys.exit(0)
    print(f"Controlling {nchairs}")

    nodelist = []
    nodelist.append(
        Node(
            namespace = "chair_0",
            package='cpmr_ch11',
            executable='leader_chair',
            name='leader_chair',
            output='screen',
            parameters=[{'chair_name' : "chair_0"}])
        )
    print(f"leaderchair done")


    for chair in range(1, nchairs):
        name = f'chair_{chair}'
        #target_name = f"chair_0"
        target_name = f'chair_{chair - 1}'
        print(f"Processing {chair}")
        nodelist.append(
            Node(
                namespace = name,
                package='cpmr_ch11',
                executable='follow_chair',
                name='follow_chair',
                output='screen',
               parameters=[{'chair_name': name, 'target_name': target_name}]) # use chair_{chair-1}
        )

    return LaunchDescription(nodelist)




ros2 service call /chair_0/startup std_srvs/srv/SetBool '{data: True}'
ros2 service call /chair_1/startup std_srvs/srv/SetBool '{data: True}'
ros2 service call /chair_2/startup std_srvs/srv/SetBool '{data: True}'
ros2 service call /chair_3/startup std_srvs/srv/SetBool '{data: True}'
ros2 service call /chair_4/startup std_srvs/srv/SetBool '{data: True}'
ros2 service call /chair_5/startup std_srvs/srv/SetBool '{data: True}'
ros2 service call /chair_6/startup std_srvs/srv/SetBool '{data: True}'
ros2 service call /chair_7/startup std_srvs/srv/SetBool '{data: True}'
ros2 service call /chair_8/startup std_srvs/srv/SetBool '{data: True}'
ros2 service call /chair_9/startup std_srvs/srv/SetBool '{data: True}'