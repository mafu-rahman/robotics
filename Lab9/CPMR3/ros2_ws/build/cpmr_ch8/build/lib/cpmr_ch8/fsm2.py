import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from enum import Enum
import rclpy
from rclpy.node import Node

class FSM_STATES(Enum):
    AT_START = 'At Start'
    MOVING_IN_SQUARE = 'Moving in Square'
    MOVING_IN_TRIANGLE = 'Moving in Triangle'
    TASK_DONE = 'Task Done'

class FSM(Node):
    def __init__(self):
        super().__init__('fsm_control')
        self.get_logger().info(f'{self.get_name()} created')

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

        # Initial conditions
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0
        self._cur_state = FSM_STATES.AT_START
        self._points_square = [[0, 0], [2, 0], [2, 2], [0, 2]]
        self._points_triangle = [[0, 0], [-2, 0], [2, -2]]
        self._point = 0
        self._run = False

    def _listener_callback(self, msg):
        pose = msg.pose.pose
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        roll, pitch, yaw = self.euler_from_quaternion(pose.orientation)
        self._cur_theta = yaw
        self._state_machine()

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion to euler roll, pitch, yaw.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _short_angle(self, angle):
        """Adjust the angle to be in the range -pi..pi"""
        if angle > math.pi:
            angle -= 2 * math.pi
        if angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _drive_to_goal(self, goal_x, goal_y, goal_theta):
        """Move the robot to a specific goal, checking distance and orientation"""
        twist = Twist()

        x_diff = goal_x - self._cur_x
        y_diff = goal_y - self._cur_y
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2)

        # Move towards goal position
        if dist > 0.15:  # Range tolerance
            heading = math.atan2(y_diff, x_diff)
            diff = self._short_angle(heading - self._cur_theta)
            twist.angular.z = diff * 0.5  # Turn to face goal
            if abs(diff) < 0.1:
                twist.linear.x = 0.2  # Move forward
            self._publisher.publish(twist)
            return False  # Still moving towards the goal
        else:
            # Goal reached, now align with the goal's heading
            diff_theta = self._short_angle(goal_theta - self._cur_theta)
            if abs(diff_theta) > 0.15:
                twist.angular.z = diff_theta * 0.5
                self._publisher.publish(twist)
                return False  # Still adjusting orientation
            self._publisher.publish(Twist())  # Stop moving
            return True  # Reached the goal

    def _do_state_at_start(self):
        """Start the FSM, initiate movement"""
        self.get_logger().info(f'In start state')
        self._cur_state = FSM_STATES.MOVING_IN_SQUARE
        self._point = 0  # Start at the first square point

    def _do_state_moving_in_square(self):
        """Move the robot through the square"""
        if self._drive_to_goal(self._points_square[self._point][0], self._points_square[self._point][1], 0):
            self._point += 1
            if self._point >= len(self._points_square):
                self._cur_state = FSM_STATES.MOVING_IN_TRIANGLE
                self._point = 0

    def _do_state_moving_in_triangle(self):
        """Move the robot through the triangle"""
        if self._drive_to_goal(self._points_triangle[self._point][0], self._points_triangle[self._point][1], 0):
            self._point += 1
            if self._point >= len(self._points_triangle):
                self._cur_state = FSM_STATES.TASK_DONE

    def _do_state_task_done(self):
        """Finish the task and log it"""
        self.get_logger().info(f'Task completed')

    def _state_machine(self):
        """State machine logic"""
        if self._cur_state == FSM_STATES.AT_START:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.MOVING_IN_SQUARE:
            self._do_state_moving_in_square()
        elif self._cur_state == FSM_STATES.MOVING_IN_TRIANGLE:
            self._do_state_moving_in_triangle()
        elif self._cur_state == FSM_STATES.TASK_DONE:
            self._do_state_task_done()

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
