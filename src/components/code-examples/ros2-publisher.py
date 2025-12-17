#!/usr/bin/env python3

"""
ROS 2 Publisher Example

This example demonstrates how to create a simple publisher node in ROS 2 using Python.
The publisher sends messages to a topic at regular intervals.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        """
        Initialize the publisher node.
        Creates a publisher for String messages on the 'topic' topic.
        Sets up a timer to publish messages at 0.5 second intervals.
        """
        super().__init__('minimal_publisher')

        # Create a publisher for String messages on the 'topic' topic with queue size 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Set timer period to 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for message numbering
        self.i = 0

    def timer_callback(self):
        """
        Callback function that executes at regular intervals.
        Creates and publishes a message with the current counter value.
        """
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to initialize and run the publisher node.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the publisher node
    minimal_publisher = MinimalPublisher()

    try:
        # Keep the node running until interrupted
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("Node interrupted by user")
    finally:
        # Clean up and shutdown
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()