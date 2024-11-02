import sys
if sys.prefix == '/opt/miniconda3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/mafu/Library/CloudStorage/GoogleDrive-mafu@my.yorku.ca/Other computers/My Laptop/E:/0.2 Fall 2024/EECS_4421_Robotics/Labs/Lab2/CPMR3/ros2_ws/src/install/cpmr_ch11'
