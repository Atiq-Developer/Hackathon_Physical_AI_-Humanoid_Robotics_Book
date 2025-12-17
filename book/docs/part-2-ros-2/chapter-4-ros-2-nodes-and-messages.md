---
sidebar_position: 4
---

# Chapter 4: Building a ROS 2 System

Building a ROS 2 system involves creating workspaces, packages, and custom message types to facilitate communication between your robot's components. This chapter will walk you through the practical steps of setting up your development environment and creating your first ROS 2 nodes.

## Creating a ROS 2 Workspace

A ROS 2 workspace is a directory where you store your ROS 2 packages. It's best practice to create a dedicated workspace for each project.

1.  **Create the workspace directory**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```
2.  **Initialize the workspace**:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    ```
3.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

## Writing a Simple Publisher and Subscriber

### Publisher Node

A publisher node sends data to a topic. Here's a basic Python example:

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
        msg.data = 'Hello ROS 2: %d' % self.i
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

### Subscriber Node

A subscriber node receives data from a topic:

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

## Creating Custom Messages

For more complex data types, you can define custom messages.

1.  **Define `.msg` file**:
    Create `msg/MyCustomMessage.msg` in your package:
    ```
    int32 id
    string name
    float32 value
    ```
2.  **Update `package.xml`**:
    Add dependencies:
    ```xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
3.  **Update `CMakeLists.txt`**:
    Add:
    ```cmake
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/MyCustomMessage.msg"
    )
    ```
    Then, you can `colcon build` and use `from your_package.msg import MyCustomMessage`.
