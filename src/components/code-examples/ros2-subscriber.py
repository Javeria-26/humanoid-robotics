#!/usr/bin/env python3

"""
ROS 2 Subscriber Example

This example demonstrates how to create a simple subscriber node in ROS 2 using Python.
The subscriber listens for messages on a topic and logs them when received.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        """
        Initialize the subscriber node.
        Creates a subscription to String messages on the 'topic' topic.
        """
        super().__init__('minimal_subscriber')

        # Create a subscription to String messages on the 'topic' topic with queue size 10
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        """
        Callback function that executes when a message is received.
        Logs the received message data.

        Args:
            msg: The received message of type std_msgs.msg.String
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize and run the subscriber node.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the subscriber node
    minimal_subscriber = MinimalSubscriber()

    try:
        # Keep the node running until interrupted
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        print("Node interrupted by user")
    finally:
        # Clean up and shutdown
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()