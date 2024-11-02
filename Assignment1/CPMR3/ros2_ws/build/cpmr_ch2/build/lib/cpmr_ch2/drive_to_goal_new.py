import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion

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

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_t', 0.0)
        self.declare_parameter('vel_gain', 5.0)  # Default velocity gain
        self.declare_parameter('max_vel', 0.4)   # Default max velocity (40 cm/sec)

        self._goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self._goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self._goal_t = self.get_parameter('goal_t').get_parameter_value().double_value
        self._vel_gain = self.get_parameter('vel_gain').get_parameter_value().double_value
        self._max_vel = self.get_parameter('max_vel').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info(f"initial goal {self._goal_x} {self._goal_y} {self._goal_t}")

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)


    # This approach applies the velocity to the magnite rather than the separately in the x and y coordinates
    def _listener_callback(self, msg, max_pos_err=0.05):
        pose = msg.pose.pose

        cur_x = pose.position.x
        cur_y = pose.position.y
        o = pose.orientation
        roll, pitch, yaw = euler_from_quaternion(o)
        cur_t = yaw
        
        x_diff = self._goal_x - cur_x
        y_diff = self._goal_y - cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)

        twist = Twist()
        if dist > max_pos_err:
            # Applying gain to the velocity
            vx = x_diff * self._vel_gain
            vy = y_diff * self._vel_gain

            # Computing the velocity magnitude
            velocity_magnitude = math.sqrt(vx**2 + vy**2)

            # Applying the velocity limit to the magnitude
            if velocity_magnitude > self._max_vel:
                scale = self._max_vel / velocity_magnitude
                vx *= scale
                vy *= scale


            twist.linear.x = vx * math.cos(cur_t) + vy * math.sin(cur_t)
            twist.linear.y = -vx * math.sin(cur_t) + vy * math.cos(cur_t)

            self.get_logger().info(f"At ({cur_x},{cur_y},{cur_t}), goal ({self._goal_x},{self._goal_y},{self._goal_t}), velocity ({vx},{vy})")
        
        self._publisher.publish(twist)

    def parameter_callback(self, params):
        self.get_logger().info(f'move_robot_to_goal parameter callback {params}')
        for param in params:
            self.get_logger().info(f'move_robot_to_goal processing {param.name}')
            if param.name == 'goal_x' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_x = param.value
            elif param.name == 'goal_y' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_y = param.value
            elif param.name == 'goal_t' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_t = param.value
            elif param.name == 'vel_gain' and param.type_ == Parameter.Type.DOUBLE:
                self._vel_gain = param.value
            elif param.name == 'max_vel' and param.type_ == Parameter.Type.DOUBLE:
                self._max_vel = param.value
            else:
                self.get_logger().warn(f'{self.get_name()} Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
            self.get_logger().info(f"Changed parameters to goal: ({self._goal_x}, {self._goal_y}, {self._goal_t}), velocity gain: {self._vel_gain}, max velocity: {self._max_vel}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
