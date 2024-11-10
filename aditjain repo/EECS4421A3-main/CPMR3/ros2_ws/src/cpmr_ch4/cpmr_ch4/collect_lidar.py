import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import cv2
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class CollectLidar(Node):
    _WIDTH = 513
    _HEIGHT = 513
    _M_PER_PIXEL = 0.05
    
    cur_X = 0
    cur_Y = 0 
    cur_Ya = 0 
    def __init__(self):
        super().__init__('collect_lidar')
        self.get_logger().info(f'{self.get_name()} created')

        self._map = np.zeros((CollectLidar._HEIGHT, CollectLidar._WIDTH), dtype=np.uint8)
        temp = self._map.shape
        self.get_logger().info(f"{temp}")
        self.create_subscription(Odometry, "/odom", self._odom_callback, 1)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 1)

    # def updateMap(self, pointToAdd):
    #     self._map[pointToAdd[0]+250,pointToAdd[1]+250] = 150

    def _scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment # 0.0174526
        ranges = msg.ranges # 360

        self.get_logger().info(f"lidar ({angle_min},{angle_max},{angle_increment},{len(ranges)})")
        self.get_logger().info(f"\n\n\n")
        for i in enumerate(ranges):

            if i[1] >0 and i[1] < 10: # makes sure sensor values are within range
                rads = math.radians(i[0]) + math.pi 
                x = (math.cos(rads) * i[1]) #- abs(CollectLidar.cur_X) 
                y = (math.sin(rads) * i[1]) #- abs(CollectLidar.cur_Y)
                self.get_logger().info(f"Cordinates: ({x+250},{y+250})")

                self._map[int(x+250),int(y+250)] = 150
                

            # if i[1] >0 and i[1] < 10: # checks to see sensor range values
                
            #     # we assume each index is a degree, since we have 360 samples
            #     # We will need to convert it into radians so we can use the Math functions
            #     rads = math.radians(i[0]) + math.pi + CollectLidar.cur_Ya # so we can have it using the normal cordinate system

            #     x = (math.cos(rads)*i[1]) - abs(CollectLidar.cur_X) # cos(theta) * hypot to get x component 
            #     y = (math.sin(rads)*i[1]) - abs(CollectLidar.cur_Y)# sin(theta) * hypot to get y component
                
            #     self.get_logger().info(f"CORDINATES: ({x},{y})")


        cv2.imshow('map',self._map)
        cv2.waitKey(10)

        angle_increment = msg.angle_increment # 0.0174526
        ranges = msg.ranges # 360
    def _odom_callback(self, msg):
        pose = msg.pose.pose
        cur_x = pose.position.x
        cur_y = pose.position.y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        cur_t = yaw # Yaw refers to rotation into the Z axis
        CollectLidar.cur_X= cur_x
        CollectLidar.cur_Y= cur_y
        CollectLidar.cur_Ya = yaw
        # self.get_logger().info(f"ODOMETRY X:{cur_x} Y:{cur_y} YAW:({cur_t})")

def main(args=None):
    rclpy.init(args=args)
    node = CollectLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

