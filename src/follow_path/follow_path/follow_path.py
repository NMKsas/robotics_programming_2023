#!/usr/bin/env python
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from math import pow, atan2, sqrt, pi
from rclpy.qos import ReliabilityPolicy, QoSProfile


# Goal constants. Replace with wanted coordinate directory.
COORDINATES_DEFAULT_DIRECTORY = "/home/sasnmk/ros/turtlebot3_ws/coordinates/"

QUIT = 1
INVALID_VALUE = "Invalid"
RETURN_LIST = 3


class PathController(Node):

    def __init__(self, goal_coordinate_list):
        # Initialize the node
        super().__init__('path_controller')
        self.get_logger().info("Initializing controller node")

        self.get_logger().info('Creating publisher for /cmd_vel topic')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Odometry is received.
        self.get_logger().info('Subscribing /odom topic')
        self.pose_subscriber = self.create_subscription(Odometry, '/odom',
                                                        self.update_pose,
                                                        QoSProfile(depth=10,
                                                                   reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info('Publishing timer period set to 1')
        self.timer = self.create_timer(1, self.move_to_point)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.x = float()
        self.y = float()
        self.goal_list = goal_coordinate_list
        self.goal_x = goal_coordinate_list[0][0]
        self.goal_y = goal_coordinate_list[0][1]
        self.counter = 1

    def update_pose(self, odom_msg):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.x = round(odom_msg.pose.pose.position.x, 4)
        self.y = round(odom_msg.pose.pose.position.y, 4)
        quaternion = odom_msg.pose.pose.orientation
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])

    def move_to_point(self):
        """Moves the turtle to the goal."""
        goal_pose = Point()
        goal_pose.x = float(self.goal_x)
        goal_pose.y = float(self.goal_y)

        inc_x = goal_pose.x - self.x
        inc_y = goal_pose.y - self.y

        # Euclidian distance
        distance_to_goal = sqrt(pow(inc_x, 2) + pow(inc_y, 2))
        # Angle to goal
        angle_to_goal = atan2(inc_y, inc_x)

        # tolerance values
        dist_tol = 0.2
        ang_tol = 0.15

        kp = 0.5
        angle_error = angle_to_goal - self.yaw

        # If the angle error is larger than pi, change turning direction, dampen speed
        if abs(angle_error) >= pi:
            if angle_error >= 0:
                angle_error -= pi
            else:
                angle_error += pi

        vel_msg = Twist()
        # turn first
        if abs(angle_error) > ang_tol:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = kp * angle_error
        else:
            # approach the goal
            if distance_to_goal > dist_tol:
                vel_msg.linear.x = kp * distance_to_goal
            else:
                self.goal_reached()
                return

        self.velocity_publisher.publish(vel_msg)

    def goal_reached(self):
        self.get_logger().info("Goal " + str(self.counter) + " reached in coordinates "
                               + str(self.goal_list[self.counter-1]))
        self.get_logger().info("Path coordinates left: " + str(self.counter) + "/" + str(len(self.goal_x)))
        self.velocity_publisher.publish(Twist())
        if len(self.goal_list) == self.counter:
            self.get_logger().info("All goals reached.")
            self.stop_turtlebot()
        else:
            self.counter += 1
            self.get_logger().info("Commencing next movement...")
            self.goal_x = self.goal_list[self.counter-1][0]
            self.goal_y = self.goal_list[self.counter-1][1]

    def stop_turtlebot(self):
        self.get_logger().info('Stopping the turtlebot')
        self.velocity_publisher.publish(Twist())
        quit()


def is_return_signal(input_value):
    string_input = str(input_value)

    if string_input.upper() == "R":
        print("Returning the coordinate list...")
        return True
    return False


def is_exit_signal(input_value):
    string_input = str(input_value)

    if string_input.upper() == "Q":
        return True
    return False


def validate_coordinate(input_value):
    try:
        coord = float(input_value)
        return coord
    except ValueError:
        print("Invalid value '" + input_value + "'! Give numeric values (e.g., 2.0)")
        return INVALID_VALUE


def enter_points_manually():
    print("-----------------------------")
    print("Entering coordinates manually")
    print("-----------------------------")
    path_points_list = []
    while True:
        print()
        print("Enter Q to quit, R to return the current list")
        print("Current coordinate list: " + str(path_points_list) + "\n")

        x_coord = input("Enter x-coordinate: ")
        if is_exit_signal(x_coord):
            return QUIT
        if is_return_signal(x_coord):
            return path_points_list

        y_coord = input("Enter y-coordinate: ")
        if is_exit_signal(y_coord):
            return QUIT
        if is_return_signal(y_coord):
            return path_points_list

        # continue to validate the inputs
        validated_x = validate_coordinate(x_coord)
        validated_y = validate_coordinate(y_coord)
        if validated_x != INVALID_VALUE and validated_y != INVALID_VALUE:
            path_points_list.append((validated_x, validated_y))


def parse_to_coordinate_list(file_path):
    coordinate_list = []
    f = open(file_path, "r")
    lines = f.readlines()

    line_count = 1
    for line in lines:
        line_content = line.strip()
        coordinates = line_content.split(",")

        if len(coordinates) != 2:
            break

        validated_x = validate_coordinate(coordinates[0])
        validated_y = validate_coordinate(coordinates[1])
        if validated_x == INVALID_VALUE or validated_y == INVALID_VALUE:
            print("Error in file, line " + str(line_count) + ": '" + line_content + "'.\n" +
                  "Coordinates must be numeric and in format <x_coordinate>,<y_coordinate>.")
            return QUIT
        else:
            coordinate_list.append((validated_x, validated_y))
        line_count += 1
    f.close()
    return coordinate_list


def read_from_file(file_directory):
    entries = os.listdir(file_directory)
    if len(entries) == 0:
        print("No files in default directory '" + COORDINATES_DEFAULT_DIRECTORY + "'!")
        return QUIT

    print("Files in the default directory: " + str(entries))
    file_name = input("Choose file: ")

    for entry in entries:
        if entry == file_name:
            return parse_to_coordinate_list(file_directory + file_name)
    print("File not found!")
    return QUIT


def is_list_empty(list_of_coordinates):
    if len(list_of_coordinates) == 0:
        print("The coordinate list is empty!")
        return True
    return False


def init_ros_node(list_of_coordinates):

    rclpy.init(args=None)

    controller = PathController(list_of_coordinates)
    try:
        while rclpy.ok():
            rclpy.spin(controller)

    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt detected.')
    rclpy.shutdown()
    print("Shutting down the controller node")


def main():
    list_of_coordinates = []
    while True:
        print("Enter M to define path manually")
        print("Enter F to read a file containing the path points")
        print("Enter Q to quit")
        choose_insertion_method = input()
        print()

        if choose_insertion_method.upper() == "M":
            list_of_coordinates = enter_points_manually()
        elif choose_insertion_method.upper() == "F":
            list_of_coordinates = read_from_file(COORDINATES_DEFAULT_DIRECTORY)

        if is_exit_signal(choose_insertion_method) or list_of_coordinates == QUIT \
                or is_list_empty(list_of_coordinates):
            print("Quitting the program...")
            return
        else:
            break

    init_ros_node(list_of_coordinates)


if __name__ == '__main__':
    main()
