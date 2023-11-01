#!/usr/bin/env python

from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from math import pow, atan2, sqrt
from rclpy.qos import ReliabilityPolicy, QoSProfile


class PathController(Node):

    def __init__(self, goal_coordinate_list):
        # Initialize the node
        super().__init__('path_controller')

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Odometry is received.
        self.pose_subscriber = self.create_subscription(Odometry, '/odom',
                                                        self.update_pose,
                                                        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
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

        dist_tol = 0.2
        ang_tol = 0.2

        angle_error = angle_to_goal - self.yaw
        kp = 0.2

        vel_msg = Twist()
        if abs(angle_error) > ang_tol:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = kp * angle_error
        else:
            if distance_to_goal > dist_tol:
                vel_msg.linear.x = kp * distance_to_goal
            else:
                self.goal_reached()

        self.velocity_publisher.publish(vel_msg)

    def goal_reached(self):
        self.get_logger().info("Goal " + str(self.counter) + " reached in coordinates "
                               + str(self.goal_list[self.counter]))
        if len(self.goal_list) == self.counter:
            self.get_logger().info("All goals reached.")
            self.stop_turtlebot()
        else:
            self.counter += 1
            self.goal_x = self.goal_list[self.counter][0]
            self.goal_y = self.goal_list[self.counter][1]

    def stop_turtlebot(self):
        self.get_logger().info('Stopping the turtlebot')
        self.velocity_publisher.publish(Twist())
        quit()
