import sys
if sys.prefix == '/Users/mafu/miniforge3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/mafu/Desktop/robotics/Assignment2/CPMR3/ros2_ws/src/cpmr_ch6/install/cpmr_ch6'
