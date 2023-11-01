import sys
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
from enum import Enum


class TopicChoice(Enum):
    CIRCLE = 'circle'
    SQUARE = 'square'


class Directions(Enum):
    UP = 'UP'
    RIGHT = 'RIGHT'
    DOWN = 'DOWN'
    LEFT = 'LEFT'


class TestPublisher(Node):

    def __init__(self, topic_choice=TopicChoice.CIRCLE, timer_period=0.5):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer_period = timer_period
        self.topic_choice = topic_choice
        self.direction = Directions.RIGHT

        if topic_choice == TopicChoice.CIRCLE:
            self.timer = self.create_timer(timer_period, self.publish_circle_message)
        else:
            # self.timer_period += 2.0
            self.timer = self.create_timer(timer_period, self.publish_square_message)

    def publish_circle_message(self):
        # Object of message type Twist()
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0

        # Publish the velocity command
        self.publisher.publish(msg)
        # log details
        self.get_logger().info('Publish twist message')

    def publish_square_message(self):

        msg = Twist()
        if self.direction == Directions.RIGHT:
            msg.linear.x = 2.0
            self.direction = Directions.DOWN
        elif self.direction == Directions.DOWN:
            msg.linear.y = -2.0
            self.direction = Directions.LEFT
        elif self.direction == Directions.LEFT:
            msg.linear.x = -2.0
            self.direction = Directions.UP
        elif self.direction == Directions.UP:
            msg.linear.y = 2.0
            self.direction = Directions.RIGHT

        self.publisher.publish(msg)
        self.get_logger().info('Publish twist message, direction is: ' + self.direction.value)

    def stop_turtlebot(self):
        # turtlebot stopped when program is interrupted
        self.get_logger().info('stopping turtlebot')
        # all velocity components to zero
        self.publisher.publish(Twist())


def main():
    print(sys.argv)
    if len(sys.argv) != 2:
        print("Publisher requires an argument: circle, square")
        sys.exit(1)
    topic_choice = TopicChoice(sys.argv[1])

    rclpy.init(args=None)

    # create publisher
    test_publisher = TestPublisher(topic_choice=topic_choice)

    try:
        # continue until interrupted
        rclpy.spin(test_publisher)

    except KeyboardInterrupt:
        # shutdown after keyboard interrupt
        test_publisher.stop_turtlebot()
        # clear the node
        test_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
