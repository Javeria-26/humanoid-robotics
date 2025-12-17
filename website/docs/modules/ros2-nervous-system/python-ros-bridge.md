---
sidebar_position: 3
---

# Python-ROS Bridge: Using rclpy

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain how the rclpy library enables Python interaction with ROS 2
- Create publisher nodes in Python to send messages to ROS 2 topics
- Create subscriber nodes in Python to receive messages from ROS 2 topics
- Implement service clients and servers using Python
- Understand the relationship between Python and ROS 2 middleware

## Introduction: Python and ROS 2

Python is one of the most popular languages in robotics and AI development. The rclpy library provides Python bindings for ROS 2, allowing Python programs to interact with the ROS 2 middleware. This creates a powerful bridge between the AI/ML ecosystem (where Python dominates) and robotic control systems.

## Understanding rclpy

rclpy is the Python client library for ROS 2. It provides the interface between Python programs and the ROS 2 middleware (DDS). With rclpy, Python developers can:

- Create and manage ROS 2 nodes
- Publish and subscribe to topics
- Make service requests and provide services
- Work with actions for long-running tasks
- Handle parameters and logging

### Installing rclpy

rclpy is part of the standard ROS 2 installation. To use it in your Python scripts, simply import it:

```python
import rclpy
from rclpy.node import Node
```

## Creating Publisher Nodes with Python

Publisher nodes in Python are created by extending the Node class and using a publisher object:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Subscriber Nodes with Python

Subscriber nodes listen to messages on specific topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing Services with Python

Services in Python involve both client and server implementations:

### Service Server

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
import sys
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Applications

The Python-ROS bridge is particularly valuable for:

1. **AI/ML Integration**: Using Python's rich AI/ML libraries (TensorFlow, PyTorch, scikit-learn) with robotic systems
2. **Rapid Prototyping**: Python's simplicity allows for quick development and testing
3. **Data Processing**: Handling sensor data with Python's data science libraries
4. **User Interfaces**: Creating GUIs and dashboards using Python frameworks

## Best Practices

- Always initialize and shutdown rclpy properly
- Use appropriate queue sizes for publishers and subscribers
- Handle exceptions and edge cases in callbacks
- Use logging for debugging and monitoring
- Follow ROS 2 naming conventions for topics, services, and nodes

## Summary

The rclpy library provides a robust bridge between Python's AI/ML ecosystem and ROS 2's robotic control capabilities. This integration enables developers to leverage Python's strengths in data processing and AI while controlling robotic systems through ROS 2's middleware architecture.

To understand how these Python-ROS interactions apply to humanoid robot models, continue to the [URDF for Humanoids](./urdf-humanoids) chapter. For a review of the fundamental ROS 2 concepts that underpin these implementations, see the [ROS 2 Fundamentals](./ros2-fundamentals) chapter.