import sys
if sys.prefix == '/Library/Frameworks/Python.framework/Versions/3.12':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/mafu/Desktop/EECS_4421_Robotics/Labs/Lab3/CPMR3/ros2_ws/src/install/cpmr_ch12'
