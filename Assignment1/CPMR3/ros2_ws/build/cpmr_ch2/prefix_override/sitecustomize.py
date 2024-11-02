import sys
if sys.prefix == '/Users/mafu/miniconda3/envs/ros2':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/mafu/Desktop/EECS_4421_Robotics/Labs/Assignment1/CPMR3/ros2_ws/install/cpmr_ch2'
