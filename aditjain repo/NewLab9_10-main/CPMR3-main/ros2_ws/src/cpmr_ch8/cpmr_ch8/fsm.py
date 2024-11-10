from enum import Enum
import math
import random
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

row_width = 6
num_rows = 4
row_offset = 0.5 
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


class FSM_STATES(Enum):
    AT_START = 'AT STart',
    HEADING_TO_TASK = 'Heading to Task',
    # need to add a task state(s)
    #==============================================
    PERFORMING_TASK = 'Cutting the grass'
    #===============================================
    RETURNING_FROM_TASK = 'Returning from Task',
    TASK_DONE = 'Task Done'

class FSM(Node):

    def __init__(self):
        super().__init__('FSM')
        self.get_logger().info(f'{self.get_name()} created')

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

        # the blackboard
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0
        self._cur_state = FSM_STATES.AT_START
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self.goalList = [[3,3,math.pi/2],[3,6,math.pi],[3.5,6,3*(math.pi/2)],[3.5,3,math.pi]] 
        self. currentGoal = self.goalList[0] 
        self.robotSpeed=0.4
        self.currentIndex =0 

    def _drive_to_goal(self, goal_x, goal_y, goal_theta):
        self.get_logger().info(f'CURRENT GOAL ({goal_x}, {goal_y})')
        self.get_logger().info(f'{self.get_name()} drive to goal')
        twist = Twist()

        x_diff = goal_x - self._cur_x
        y_diff = goal_y - self._cur_y
        dist = x_diff * x_diff + y_diff * y_diff
        self.get_logger().info(f'{self.get_name()} {x_diff} {y_diff}')

        # turn to the goal
        heading = math.atan2(y_diff, x_diff)
        if abs(self._cur_theta - heading) > math.pi/20: 
            if heading > self._cur_theta:
                twist.angular.z = 0.10
            else:
               twist.angular.z = -0.10
            self.get_logger().info(f'{self.get_name()} turning towards goal')
            self._publisher.publish(twist)
            return False

        # since we are now pointing to the right direction, go there
        if dist > 0.1*0.1:
            twist.linear.x = self.robotSpeed 
            self._publisher.publish(twist)
            self.get_logger().info(f'{self.get_name()} driving to goal {dist}')
            return False
        if abs(goal_theta - self._cur_theta) > math.pi/20:
            if goal_theta > self._cur_theta:
                self.get_logger().info(f'{self.get_name()} turning to goal direction')
                twist.angular.z = 0.005# * random.randint(1, 10) 
            else:
                twist.angular.z = -0.005# * random.randint(1, 10)
                self.get_logger().info(f'{self.get_name()} turning to goal direction')
                self._publisher.publish(twist)
        self.get_logger().info(f'{self.get_name()} at goal pose')
        return True
        
        

    def _do_state_at_start(self):
        self.get_logger().info(f'{self.get_name()} in start state')

        now = self.get_clock().now().nanoseconds * 1e-9
        if now > (self._start_time + 2):
            # once the 2 seconds have passed, lets head to our task
            self._cur_state = FSM_STATES.HEADING_TO_TASK
            
    def _do_state_heading_to_task(self):
        self.get_logger().info(f'{self.get_name()} heading to task {self._cur_x} {self._cur_y} {self._cur_theta}')
        if self._drive_to_goal(3, 3, math.pi/2):
            self._cur_state = FSM_STATES.PERFORMING_TASK
    def _do_state_performing_task(self):
        isAtGoal = self._drive_to_goal(*self.currentGoal)
        # self.get_logger().info(f'{self.currentGoal} \n')
        x=0
        y=1
        # self.log
        
        if isAtGoal:
            if self.currentGoal[x] == 3.5 and self.currentGoal[y] == 3:
                self.get_logger().info(f'{self.get_name()} completed mowing grass')
                self._cur_state = FSM_STATES.RETURNING_FROM_TASK
            
            elif self.currentGoal == self.goalList[3]:
                self.get_logger().info(f'{self.get_name()} Turning to next row')
                for goal in self.goalList:
                    goal[x] += 1
                self.currentGoal = self.goalList[0]
            else:
                self._cur_state = FSM_STATES.PERFORMING_TASK
                self.get_logger().info(f'{self.get_name()} mowing the row')
                index = self.goalList.index(self.currentGoal)
                self.currentGoal = self.goalList[index+1] 
        
        
    def _do_state_returning_from_task(self):
        self.get_logger().info(f'{self.get_name()} returning from task ')
        if self._drive_to_goal(0, 0, math.pi/2):
            self._cur_state = FSM_STATES.TASK_DONE

    def _do_state_task_done(self):
        self.get_logger().info(f'{self.get_name()} task done')

    def _state_machine(self):
        if self._cur_state == FSM_STATES.AT_START:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.HEADING_TO_TASK:
            self._do_state_heading_to_task()
        elif self._cur_state == FSM_STATES.PERFORMING_TASK:
            self._do_state_performing_task()
        elif self._cur_state == FSM_STATES.RETURNING_FROM_TASK:
            self._do_state_returning_from_task()
        elif self._cur_state == FSM_STATES.TASK_DONE:
            self._do_state_task_done()
        else:
            self.get_logger().info(f'{self.get_name()} bad state {self.state_cur_state}')

    def _listener_callback(self, msg):
        pose = msg.pose.pose

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = yaw
        self._state_machine()



def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
