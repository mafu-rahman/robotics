import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import cv2

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

    def __init__(self):
        super().__init__('collect_lidar')
        self.get_logger().info(f'{self.get_name()} created')

        self._map = np.zeros((CollectLidar._HEIGHT, CollectLidar._WIDTH), dtype=np.uint8)

        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_yaw = 0.0

        self.create_subscription(Odometry, "/odom", self._odom_callback, 1)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 1)

    def _scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        for i, r in enumerate(ranges):
            if r < msg.range_max:
                angle = angle_min + i * angle_increment

                # Converting from LIDAR polar coordinates to Cartesian coordinates
                x_car = r * math.cos(angle)
                y_car = r * math.sin(angle)

                # Transform from robot-relative cartesian coordinates to world coordinates
                x_world = self._cur_x + x_car * math.cos(self._cur_yaw) - y_car * math.sin(self._cur_yaw)
                y_world = self._cur_y + x_car * math.sin(self._cur_yaw) + y_car * math.cos(self._cur_yaw)

                # Convert world coordinates to grid coordinates
                grid_x = int(x_world / CollectLidar._M_PER_PIXEL + CollectLidar._WIDTH // 2)
                grid_y = int(y_world / CollectLidar._M_PER_PIXEL + CollectLidar._HEIGHT // 2)

                # Mark the grid cell as occupied if within bounds
                if 0 <= grid_x < CollectLidar._WIDTH and 0 <= grid_y < CollectLidar._HEIGHT:
                    self._map[grid_y, grid_x] = 255  # Occupied

        cv2.imshow('map', self._map)
        cv2.waitKey(10)

    def _odom_callback(self, msg):
        pose = msg.pose.pose
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        _, _, self._cur_yaw = euler_from_quaternion(pose.orientation)
        self.get_logger().info(f"Robot at ({self._cur_x}, {self._cur_y}, {self._cur_yaw})")

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