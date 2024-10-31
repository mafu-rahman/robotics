import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped

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


class move_to_goal(Node):
    def __init__(self):
        super().__init__('aruco_robot_controller')

        # Publisher for cmd_vel to control robot velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to know if the target is visible
        self.create_subscription(Bool, '/target_visible', self.target_visible_callback, 10)

        # Subscriber to get the position and orientation of the target
        self.create_subscription(TransformStamped, '/target_pose', self.target_pose_callback, 10)

        # Subscriber to get the odometry data
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.target_visible = False  # Whether the target is visible or not
        self.target_distance = None  # Distance to the target (None if not visible)
        self.stop_distance = 0.01  # Stop 1 cm away from the target

        self.robot_pose = None  # Robot's current pose
        self.robot_yaw = None  # Robot's current yaw angle
        self.target_position = None  # Target's position (x, y)

        self.timer = self.create_timer(0.1, self.control_loop)  # Timer for control loop

    def target_visible_callback(self, msg):
        self.target_visible = msg.data

    def target_pose_callback(self, msg):
        # Assuming the target's pose is relative to the robot's coordinate frame
        # Translation (x, y, z) gives the position of the target
        self.target_position = (msg.transform.translation.x, msg.transform.translation.y)
        self.target_distance = math.sqrt(msg.transform.translation.x ** 2 + msg.transform.translation.y ** 2)

    def odom_callback(self, msg):
        # Extract position and orientation (yaw) from odometry data
        self.robot_pose = msg.pose.pose.position
        _, _, self.robot_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def control_loop(self):
        if self.robot_pose is None or self.robot_yaw is None:
            self.get_logger().info('Waiting for robot odometry...')
            return

        twist = Twist()

        if not self.target_visible:
            # Target is not visible, spin the robot
            twist.angular.z = 0.5  # Rotate at 0.5 rad/s
            self.get_logger().info('Spinning to search for target...')
        elif self.target_visible and self.target_position is not None:
            target_x, target_y = self.target_position
            self.get_logger().info(f'Target visible at distance {self.target_distance:.2f}m')

            # Calculate angle to the target
            angle_to_target = math.atan2(target_y, target_x)

            # Adjust the angle based on the robot's current orientation (yaw)
            angle_difference = angle_to_target - self.robot_yaw

            # Normalize the angle difference to the range [-pi, pi]
            angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi

            if self.target_distance > self.stop_distance:
                # Move forward and adjust angular velocity to turn towards the target
                twist.linear.x = 0.2  # Move forward at 0.2 m/s
                twist.angular.z = 0.5 * angle_difference  # Proportional control to turn towards target
                self.get_logger().info('Approaching the target...')
            else:
                # Stop the robot when within 1 cm distance
                twist.linear.x = 0.0  # Stop moving forward
                twist.angular.z = 0.0  # Stop rotating
                self.get_logger().info('Stopping the robot 1cm in front of the target.')

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = move_to_goal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
