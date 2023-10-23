#!/usr/bin/env python

import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import JointState
from math import pow, atan2, sqrt
from rclpy.qos import ReliabilityPolicy, QoSProfile

# Goal constants
X = 0
Y = 1


class TurtleBotController(Node):

    def __init__(self, goal_x, goal_y, goal_angle=0.0):
        # Initialize the node
        super().__init__('tb3_controller')

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Odometry is received.
        self.pose_subscriber = self.create_subscription(Odometry, '/odom',
                                                        self.update_pose,
                                                        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(1, self.move_to_pose)

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
        angle_to_goal = atan2(inc_y,inc_x)
        #log_msg = 'DTG : {:.3f} ATG {:.3f}'.format(distance_to_goal,angle_to_goal)
        #self.get_logger().info(log_msg)

        dist_tol = 0.2
        ang_tol = 0.2

        angle_error = angle_to_goal - self.yaw
        kp = 0.5

        vel_msg = Twist()
        if abs(angle_error) > ang_tol:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = kp * angle_error
        else:
            if distance_to_goal > dist_tol:
                vel_msg.linear.x = kp * distance_to_goal
            else:
                self.velocity_publisher.publish(Twist())
                self.get_logger().info("Goal reached")
                quit()
        print(vel_msg)
        self.velocity_publisher.publish(vel_msg)


    def move_to_pose(self):

        goal_pose = Point()
        goal_pose.x = float(self.goal_x)
        goal_pose.y = float(self.goal_y)

        inc_x = goal_pose.x - self.x
        inc_y = goal_pose.y - self.y

        # Euclidian distance
        distance_to_goal = sqrt(pow(inc_x, 2) + pow(inc_y, 2))
        # Angle to goal
        angle_to_point = atan2(inc_y,inc_x)
        #log_msg = 'DTG : {:.3f} ATG {:.3f}'.format(distance_to_goal,angle_to_point)
        #self.get_logger().info(log_msg)

        dist_tol = 0.2
        ang_tol = 0.2

        angle_error = angle_to_point - self.yaw
        kp = 0.5

        vel_msg = Twist()
        if abs(angle_error) > ang_tol:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = kp * angle_error
        else:
            if distance_to_goal > dist_tol:
                vel_msg.linear.x = kp * distance_to_goal
            else:
                if (self.goal_angle != 0.0):

                self.get_logger().info("Goal reached")
                self.stop_turtlebot()

        print(vel_msg)
        self.velocity_publisher.publish(vel_msg)

    def turn_to_angle(self):
        angle_error = self.goal_angle - self.yaw
        vel_msg = Twist()

        if abs(angle_error) > ang_tol:
            vel_msg.linear.x = 0.0
            vel_msg.linear.x = kp * angle_error
        else:
            self.get_logger().info("Goal angle reached")
            self.stop_turtlebot()


    def stop_turtlebot(self):
        self.get_logger().info('Stopping the turtlebot')
        self.velocity_publisher.publish(Twist())


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

    controller = TurtleBotController(goal_x, goal_y)
    try:
        while rclpy.ok():
            rclpy.spin(controller)

    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt detected.')

    controller.stop_turtlebot()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
