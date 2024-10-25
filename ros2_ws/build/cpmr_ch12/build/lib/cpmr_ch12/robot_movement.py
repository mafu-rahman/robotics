import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import time


def hello():
    print("Hello")




def main(args=None):
    rclpy.init(args=args)
    gesture_based_arm_control = RobotMovement()
    try:
        rclpy.spin(gesture_based_arm_control)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
