import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


import cv2
import json
import random
from datetime import datetime

WORLD_WIDTH = 2800
WORLD_HEIGHT = 3500

N = 5000  # Number of random samples
BACKGROUND_COLOR = (255, 255, 255)  # White background
OCCUPIED_COLOR = (0, 0, 0)  # Black for obstacles

ROBOT_RADIUS = 0.5  
PIXEL_RESOLUTION = 0.1  # Each pixel represents 0.1 meters

def euler_from_quaternion(quaternion):
    """Converts quaternion (w in last place) to euler roll, pitch, yaw"""
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

def load_image():    #loads the image and returns dilated the world
    image_path = 'blueprint_model.jpg'
    blueprint_img = cv2.imread(image_path)
    
    gray_img = cv2.cvtColor(blueprint_img, cv2.COLOR_BGR2GRAY)
    _, thresholded_img = cv2.threshold(gray_img, 120, 255, cv2.THRESH_BINARY_INV)
    world = np.full((WORLD_HEIGHT, WORLD_WIDTH, 3), BACKGROUND_COLOR, dtype=np.uint8)
    world[thresholded_img == 0] = (0, 0, 0)

    robot_radius_pixels = int(ROBOT_RADIUS / PIXEL_RESOLUTION)
    kernel_size = robot_radius_pixels * 2 + 1  
    kernel = np.ones((kernel_size, kernel_size), np.uint8)

    dilated_world = cv2.dilate(world, kernel, iterations=1)
    
    dilated_world = cv2.bitwise_not(dilated_world)

    return dilated_world

def seed_random_points(world, num_samples):
    """Randomly seed the world with points and mark them based on occupancy."""

    world_with_points = world.copy()  # Create a copy of the world image
    # List to hold points that are not in an obstacle
    free_points = []

    for _ in range(num_samples):
        # Generate random (x, y) coordinates
        x = random.randint(0, WORLD_WIDTH - 1)
        y = random.randint(0, WORLD_HEIGHT - 1)

        # Check if the point is in an obstacle by seeing if it's black
        if np.array_equal(world_with_points[y, x], BACKGROUND_COLOR):  # Obstacle check
            cv2.circle(world_with_points, (x, y), 6, (0, 255, 0), -1)  # Mark as red if in obstacle
            free_points.append((x, y))  # Add to free points list

    return world_with_points, free_points

def is_line_free(world, point1, point2, step_size=1):
    """Check if the line between two points is free of obstacles."""
    x1, y1 = point1
    x2, y2 = point2

    # Calculate the total distance between the points
    distance = math.dist(point1, point2)

    # Calculate the number of steps based on the step size
    num_steps = int(distance / step_size)

    # Calculate the increments for each step
    x_step = (x2 - x1) / num_steps
    y_step = (y2 - y1) / num_steps

    # Iterate through each step along the line
    for i in range(num_steps + 1):  # +1 to include the endpoint
        # Calculate the current point along the line
        x = int(x1 + i * x_step)
        y = int(y1 + i * y_step)

        # Check if the current point is within an obstacle
        if np.array_equal(world[y, x], [0, 0, 0]):  # Black indicates an obstacle
            return False

    # If no obstacles were encountered, return True
    return True

def grow_rrt(world, free_points):
    """Grow the RRT tree by connecting free points."""

    tree_image = world.copy()  # Create a copy of the world image for the tree

    # Choose a random point as the root
    root = random.choice(free_points)
    tree = [root]
    edges = []  # List to store edges between nodes

    for point in free_points:
        if point == root:
            continue

        # Find the nearest point in the tree to this point
        nearest_point = min(tree, key=lambda node: math.dist(node, point))

        # Check if a straight line can connect without crossing obstacles
        if is_line_free(tree_image, nearest_point, point):
            cv2.line(tree_image, nearest_point, point, (0, 255, 0), 1)  # Green line for connection
            tree.append(point)
            edges.append((nearest_point, point))

    return tree, edges, tree_image

def connect_start_and_goal(world, tree, edges, start, goal):
    """Attempt to connect the start and goal to the tree."""
    # Create a copy of the image for visualization
    connect_image = world.copy()
    
    # Attempt to connect start to the tree
    start_connected = False
    for node in tree:
        if is_line_free(world, start, node):
            edges.append((start, node))
            start_connected = True
            cv2.line(connect_image, start, node, (255, 0, 0), 1)  # Blue line for start connection
            break

    # Attempt to connect goal to the tree
    goal_connected = False
    for node in tree:
        if is_line_free(world, goal, node):
            edges.append((goal, node))
            goal_connected = True
            cv2.line(connect_image, goal, node, (0, 255, 255), 1)  # Yellow line for goal connection
            break

    return edges, connect_image, start_connected, goal_connected

def find_shortest_path(edges, start, goal):
    """Find the shortest path in the tree from start to goal using BFS."""
    from collections import deque, defaultdict
    
    # Build an adjacency list for the tree
    adj_list = defaultdict(list)
    for u, v in edges:
        adj_list[u].append(v)
        adj_list[v].append(u)

    # BFS to find the shortest path
    queue = deque([[start]])
    visited = set([start])
    while queue:
        path = queue.popleft()
        node = path[-1]
        if node == goal:
            return path
        for neighbor in adj_list[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(path + [neighbor])
    return None

def display_map(name, world):
    cv2.imshow(name, world)
    cv2.waitKey(10)
   
def image_to_gazebo_coordinates(path, image_width=WORLD_WIDTH, image_height=WORLD_HEIGHT, scale_factor=0.01):
    """Converts image coordinates to Gazebo coordinates."""
    gazebo_path = []
    for image_x, image_y in path:
        gazebo_x = (image_x - image_width / 2) * scale_factor
        gazebo_y = (image_height / 2 - image_y) * scale_factor  # Invert y-axis and scale
        gazebo_path.append((gazebo_x, gazebo_y))
    return gazebo_path

def gazebo_to_image_coordinates(gazebo_x, gazebo_y, image_width=2800, image_height=3500, scale_factor=100):
    image_x = (gazebo_x * scale_factor) + image_width / 2 
    image_y = (image_height / 2) - (gazebo_y * scale_factor)
    return (int(image_x), int(image_y))

class FindPath(Node):
    def __init__(self):
        super().__init__('find_path')
        self.get_logger().info(f'{self.get_name()} created')
        
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0
        self._current_waypoint_index = 0

        
        ### World Variables ###
        self._tree, self._edges, self._tree_image = self.create_rrt_world()
        self._waypoints = [(0, 0)]

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

        self.create_service(SetBool, '/startup', self._startup_callback)
        self._run = False

    def create_rrt_world(self):
        world = load_image()
        self.get_logger().info(f'world map created')
        #display_map("world", world)

        # Seed random points and retrieve free points
        world_with_points, free_points = seed_random_points(world, N)
        self.get_logger().info(f'world map with points created')
        #display_map("world with points", world_with_points)

        # Grow the RRT tree from the free points and display it
        tree, edges, tree_image = grow_rrt(world_with_points, free_points)
        self.get_logger().info(f'trees created')
        display_map("world", tree_image)

        return tree, edges, tree_image

    def process_path(self):  # returns the path in gazebo coordinates
        start = gazebo_to_image_coordinates(self._cur_x, self._cur_y)
        goal = (2000, 3000)

        cv2.circle(self._tree_image, start, 14, (0, 165, 255), -1) #Orange
        cv2.circle(self._tree_image, goal, 14, (255, 192, 203), -1) #Pink

        edges, connect_image, start_connected, goal_connected = connect_start_and_goal(self._tree_image, self._tree, self._edges, start, goal)
        path = find_shortest_path(edges, start, goal)

        if not start_connected or not goal_connected:
            self.get_logger().info(f"Either start or goal could not be connected to the tree.")
            self._run = False
            return None
        
        else:
            path_image = connect_image.copy()
            if path:
                for i in range(len(path) - 1):
                    cv2.line(path_image, path[i], path[i + 1], (255, 0, 255), 2)  # Magenta line for path
                self.get_logger().info(f'path created')
                display_map("path image", path_image)
                path = image_to_gazebo_coordinates(path)

        return path

    def _listener_callback(self, msg):
        """Callback to update the robot's current position and drive towards the next waypoint."""
        pose = msg.pose.pose
        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = self._short_angle(yaw)

        self.get_logger().info(f"x: {self._cur_x} y: {self._cur_y}")        

        if self._run and (self._current_waypoint_index < len(self._waypoints)):
            target_x, target_y = self._waypoints[self._current_waypoint_index]
            self._drive_to_goal(target_x, target_y)

    def _short_angle(self, angle):
        """Normalize an angle to be within the range [-pi, pi]."""
        if angle > math.pi:
            angle = angle - 2 * math.pi
        if angle < -math.pi:
            angle = angle + 2 * math.pi
        return angle

    def _compute_speed(self, diff, max_speed, min_speed, gain):
        """Compute the speed based on the difference."""
        speed = abs(diff) * gain
        speed = min(max_speed, max(min_speed, speed))
        return math.copysign(speed, diff)

    def _drive_to_goal(self, goal_x, goal_y, heading_tol=0.15, range_tol=0.15):
        """Drive the robot to the target goal."""
        twist = Twist()

        x_diff = goal_x - self._cur_x
        y_diff = goal_y - self._cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)

        if dist > range_tol:
            self.get_logger().info(f'{self.get_name()} driving to goal x: {goal_x} y: {goal_y}')
            heading = math.atan2(y_diff, x_diff)
            diff = self._short_angle(heading - self._cur_theta)

            if abs(diff) > heading_tol:
                twist.angular.z = self._compute_speed(diff, 0.6, 0.5, 0.2)
                # self.get_logger().info(f'{self.get_name()} turning towards goal heading {heading} current {self._cur_theta} diff {diff}')
                self._publisher.publish(twist)
                return False

            twist.linear.x = self._compute_speed(dist, 0.6, 0.3, 0.2)
            self._publisher.publish(twist)
            # self.get_logger().info(f'{self.get_name()} moving forward, distance: {dist}')
            return False
        else:
            self.get_logger().info(f'{self.get_name()} reached waypoint ({goal_x}, {goal_y})') # Reached goal, move to next waypoint
            self._current_waypoint_index += 1

            if self._current_waypoint_index >= len(self._waypoints):
                self.get_logger().info('All waypoints reached!')
                return True
            return False

    def _startup_callback(self, request, resp):
            self.get_logger().info(f'Got a request {request}')        
            if request.data:
                self._waypoints = self.process_path() 
                if(self._waypoints != None):
                    self.get_logger().info(f'robot starting')
                    self._run = True
                    resp.success = True
                    resp.message = "Architecture running"

            else:
                self.get_logger().info(f'robot suspended')
                self._run = False
                resp.success = True
                resp.message = "Architecture suspended"
            return resp

def main(args=None):
    rclpy.init(args=args)
    node = FindPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

