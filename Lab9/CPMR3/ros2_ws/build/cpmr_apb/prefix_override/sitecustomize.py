import sys
if sys.prefix == '/Users/mafu/miniforge3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/mafu/Desktop/robotics/Lab9/CPMR3/ros2_ws/install/cpmr_apb'
