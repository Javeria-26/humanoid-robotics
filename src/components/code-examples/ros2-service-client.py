#!/usr/bin/env python3

"""
ROS 2 Service Client Example

This example demonstrates how to create a service client in ROS 2 using Python.
The client sends a request to a service and waits for the response.
Note: This example assumes there's a service called 'add_two_ints' available.
"""

import sys
import rclpy
from rclpy.node import Node

# Import the service type (this would normally be generated from a .srv file)
# For this example, we'll use a mock service type
# In a real scenario, you would import from your service definition:
# from your_package.srv import AddTwoInts
from example_interfaces.srv import AddTwoInts


class MinimalClientAsync(Node):
    def __init__(self):
        """
        Initialize the service client node.
        Creates a client for the 'add_two_ints' service.
        """
        super().__init__('minimal_client_async')

        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to the service and wait for the response.

        Args:
            a: First integer to add
            b: Second integer to add

        Returns:
            The response from the service
        """
        # Set the request parameters
        self.request.a = a
        self.request.b = b

        # Make an asynchronous service call
        self.future = self.cli.call_async(self.request)

        # Wait for the response
        rclpy.spin_until_future_complete(self, self.future)

        # Return the result
        return self.future.result()


def main(args=None):
    """
    Main function to initialize and run the service client node.
    Takes two command-line arguments as integers to add together.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Check if two arguments were provided
    if len(sys.argv) != 3:
        print("Usage: ros2 run package_name ros2_service_client.py <int_a> <int_b>")
        print("Example: ros2 run my_package ros2_service_client.py 2 3")
        return

    # Create the service client node
    minimal_client = MinimalClientAsync()

    try:
        # Parse the command-line arguments as integers
        a = int(sys.argv[1])
        b = int(sys.argv[2])

        # Send the request and get the response
        response = minimal_client.send_request(a, b)

        # Log the result
        minimal_client.get_logger().info(
            f'Result of add_two_ints: {a} + {b} = {response.sum}'
        )
    except ValueError:
        print("Please provide two integers as arguments")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Clean up and shutdown
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()