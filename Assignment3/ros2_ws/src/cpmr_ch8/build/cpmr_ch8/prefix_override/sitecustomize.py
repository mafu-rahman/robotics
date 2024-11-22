import sys
if sys.prefix == '/Users/mafu/miniforge3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/mafu/Desktop/robotics/Assignment3/ros2_ws/src/cpmr_ch8/install/cpmr_ch8'
