#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from math import pow, atan2, sqrt
from rclpy.qos import ReliabilityPolicy, QoSProfile

# Goal constants
X = 0
Y = 1


class TurtleBotController(Node):

    def __init__(self, goal_x, goal_y, goal_angle=None):
        # Initialize the node
        super().__init__('tb3_controller')

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
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_angle = goal_angle

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
                if self.goal_angle is not None:
                    self.turn_to_angle()
                self.stop_turtlebot()
        self.velocity_publisher.publish(vel_msg)

    def turn_to_angle(self):
        angle_error = self.goal_angle - self.yaw
        vel_msg = Twist()

        ang_tol = 0.2
        kp = 0.5

        if abs(angle_error) > ang_tol:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = kp * angle_error
            self.velocity_publisher.publish(vel_msg)
        else:
            self.get_logger().info("Goal angle reached")

    def stop_turtlebot(self):
        self.get_logger().info('Stopping the turtlebot')
        self.velocity_publisher.publish(Twist())
        quit()


def main(args=None):

    rclpy.init(args=args)

    # Get the input from the user.
    try:
        goal_x = float(input("Set your x goal: "))
        goal_y = float(input("Set your y goal: "))
    #    tolerance = float(input("Set your tolerance: "))
    except ValueError:
        print("Invalid value! Give values in float format, using decimals (e.g., 2.0)")
        return
    try:
        goal_angle = float(input("Set your goal angle (optional): "))
    except ValueError:
        goal_angle = None

    controller = TurtleBotController(goal_x, goal_y, goal_angle)
    try:
        while rclpy.ok():
            rclpy.spin(controller)

    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt detected.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
