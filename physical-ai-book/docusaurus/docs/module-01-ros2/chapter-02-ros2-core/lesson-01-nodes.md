---
id: lesson-01-nodes
title: "Lesson 1: Nodes - The Building Blocks of ROS 2"
sidebar_position: 1
description: Learning to create and run a basic ROS 2 Node, the fundamental executable in a ROS system.
---

### Lesson Objective
To write and run a minimal "Hello World" ROS 2 Node using the `rclpy` Python library.

### Prerequisites
- Ubuntu 22.04 with ROS 2 Humble installed.
- Visual Studio Code (or your preferred text editor).
- A basic understanding of Python.

### Concept Explanation
A **Node** is the fundamental building block of a ROS 2 system. Think of it as a single, self-contained program that performs a specific task. You might have one node for controlling the wheels, another for reading camera data, and a third for planning a path.

The power of ROS is that these nodes can be written in different languages (Python, C++, etc.), run on different computers, and they can all communicate with each other seamlessly. The collection of all running nodes and their connections is called the **ROS Graph**.

To create a node in Python, you use the `rclpy` (ROS Client Library for Python). The basic steps are:
1.  **Initialize `rclpy`**: This connects your program to the ROS 2 system.
2.  **Create a Node**: You create an instance of the `Node` class.
3.  **Do Work**: The node can then publish messages, subscribe to topics, etc. We'll do this in later lessons. For now, it will just print a message.
4.  **Spin the Node**: You call `rclpy.spin()` to keep the node alive and responsive to ROS 2 events.
5.  **Shutdown**: When the program is closed (e.g., with Ctrl+C), you clean up and shut down the node.

### Real-World Analogy
Think of a busy restaurant kitchen. Each chef is a **Node**. One chef might only chop vegetables (a specific task). Another might only work the grill. They are separate "programs," but they are all part of the same kitchen (the **ROS Graph**). They communicate by putting dishes on a central counter or calling out orders (this is what we'll learn about in the next lesson on Topics).

### Hands-On Task
**Task**: Create and run your first ROS 2 Node.

1.  **Create a Workspace**: Open a terminal and create a directory for your ROS 2 projects.
    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    ```
2.  **Create a Python File**: Inside the `src` directory, create a new file named `my_first_node.py`.
3.  **Write the Code**: Copy the Python code from the example below into `my_first_node.py`.
4.  **Run the Node**:
    - Open a new terminal.
    - Source your ROS 2 installation: `source /opt/ros/humble/setup.bash`
    - Navigate to the `src` directory: `cd ros2_ws/src`
    - Run the Python script: `python3 my_first_node.py`
    - You should see the output "Hello from my_first_node!" printed repeatedly.
    - Press `Ctrl+C` to stop the node.

### Python + ROS 2 Code Example
```python
# my_first_node.py

import rclpy
from rclpy.node import Node

class MyNode(Node):
    """
    A simple ROS 2 Node that prints a message periodically.
    """
    def __init__(self):
        super().__init__('my_first_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Hello from my_first_node!')

    def timer_callback(self):
        """
        This function is called every second by the timer.
        """
        self.get_logger().info('Node is alive...')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    my_node = MyNode()

    # Spin the node so the timer can run
    rclpy.spin(my_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    
    # Shutdown the rclpy library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Forgetting to `source` the ROS 2 setup file in your terminal. If you get a "command not found" error for `rclpy` or other ROS commands, this is usually the reason.
- **Error**: `ModuleNotFoundError: No module named 'rclpy'`
- **Solution**: Make sure you have sourced your ROS 2 installation: `source /opt/ros/humble/setup.bash`.
- **Tip**: The `get_logger().info()` method is the standard way to print information in a ROS 2 node. It's better than `print()` because it includes timestamps and the node name automatically.

### Mini Assessment
1.  What is a ROS 2 Node?
    a) A physical computer.
    b) A self-contained program that performs a specific task.
    c) A type of sensor.
2.  What is the name of the Python library for interacting with ROS 2?
    a) rospy
    b) rclpy
    c) ros_python
3.  What is the purpose of `rclpy.spin()`?
    a) To rotate the robot.
    b) To keep the node running and responsive.
    c) To stop the node.
4.  In the code example, what does `self.create_timer(1.0, self.timer_callback)` do?
    a) It stops the node after 1 second.
    b) It calls the `timer_callback` function once.
    c) It calls the `timer_callback` function every 1 second.
5.  What is the ROS Graph?
    a) A chart of robot performance.
    b) A picture of the robot.
    c) The collection of all running nodes and their connections.