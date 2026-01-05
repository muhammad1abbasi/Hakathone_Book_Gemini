---
id: lesson-01-python-to-ros
title: "Lesson 1: Structuring Nodes with Python Classes"
sidebar_position: 1
description: Move beyond simple scripts and learn to structure your ROS 2 nodes professionally using Python classes.
---

### Lesson Objective
To refactor our simple publisher and subscriber nodes into a more organized and scalable structure using Python classes.

### Prerequisites
- Completion of Chapter 2.
- A solid understanding of basic Python classes.

### Concept Explanation
So far, we've written our nodes as standalone scripts. This is fine for simple examples, but as our nodes get more complex, this becomes messy. The standard and most effective way to write a ROS 2 node is to encapsulate its logic within a Python class.

By creating a class that inherits from `rclpy.node.Node`, we can:
1.  **Organize State**: Store variables related to the node (like publishers, subscribers, timers, and counters) as class attributes (e.g., `self.publisher_`). This is much cleaner than using global variables.
2.  **Organize Logic**: Group related functions as methods of the class. For example, a timer's callback becomes a method.
3.  **Improve Reusability**: A well-structured node class can be more easily imported and used in other parts of your project.
4.  **Manage Lifecycle**: The `__init__` method becomes the natural place to set up all your publishers, subscribers, etc., when the node is created.

The `main` function of your script then becomes very simple: it initializes `rclpy`, creates an instance of your node class, spins it, and then handles shutdown. The core logic is all neatly contained within the class.

### Real-World Analogy
Think of building with LEGOs.
- **Simple Scripts** are like having a big pile of loose LEGO bricks. You can build something, but it's disorganized, hard to find the piece you need, and difficult to modify.
- **Class-Based Nodes** are like having a pre-built LEGO component, like a car chassis. The wheels, axles, and base are all neatly assembled (`__init__`). It has clear connection points (`self.publisher_`) and defined behaviors (`timer_callback` method). You can easily snap this component into a larger creation without worrying about its internal details.

### Hands-On Task
**Task**: Refactor the publisher and subscriber from the previous chapter into classes.

*Our previous examples already used a class-based structure because it is the standard best practice!* This lesson serves to formalize why we did it that way. Let's review the structure of our `SimplePublisher` from the previous lesson and highlight the key organizational benefits.

1.  **Review the Code**: Open your `simple_publisher.py` file from the last chapter.
2.  **Identify the Components**:
    - **Inheritance**: Notice `class SimplePublisher(Node):`. We are inheriting all the powerful base functionality from the `Node` class.
    - **State Management**: In `__init__`, we store the publisher, timer, and counter `i` as `self.publisher_`, `self.timer`, and `self.i`. They are neatly tied to the node instance.
    - **Logic Grouping**: The `timer_callback` is a method of the class, not a standalone function. It has access to the node's state via `self`.
    - **Clean Entrypoint**: The `main` function is clean and only handles the ROS 2 boilerplate, not the application logic.

### Python + ROS 2 Code Example (Review)
```python
# simple_publisher.py (Annotated for review)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# By inheriting from Node, we get all the ROS 2 node capabilities.
class SimplePublisher(Node):
    def __init__(self):
        # The __init__ method is our setup function.
        super().__init__('simple_publisher')
        
        # State (attributes) are stored on the instance using `self`.
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    # Logic is organized into methods.
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        
        # We can access our state using `self`.
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

# The main function is just the "runner" or "driver".
def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of our organized class.
    simple_publisher = SimplePublisher()
    
    # Spin it.
    rclpy.spin(simple_publisher)
    
    # Clean up.
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Forgetting `self` when accessing attributes or methods (e.g., calling `publisher_.publish()` instead of `self.publisher_.publish()`). This will result in a `NameError`.
- **Mistake**: Forgetting to call `super().__init__('node_name')` at the beginning of your class's `__init__` method. This is essential to properly initialize the underlying `Node` class. If you forget this, nothing will work.
- **Tip**: Adopt this class-based structure as your default for all nodes, no matter how simple. It's a habit that will pay huge dividends as your projects grow.

### Mini Assessment
1.  Why is it better to use a class for a ROS 2 node?
    a) It runs faster.
    b) It helps organize the node's state and logic.
    c) It's the only way to create a publisher.
2.  In a class-based node, where is the best place to create your publishers and subscribers?
    a) In the `main` function.
    b) In a separate setup script.
    c) In the `__init__` method of the class.
3.  How do you access a node's publisher if it was defined as `self.publisher_` in `__init__`?
    a) `publisher_`
    b) `node.publisher_`
    c) `self.publisher_`
4.  What must be the first line inside your node class's `__init__` method?
    a) `rclpy.init()`
    b) `super().__init__('your_node_name')`
    c) `self.create_publisher(...)`
5.  What is the primary role of the `main` function in this pattern?
    a) To contain all the publishing and subscribing logic.
    b) To handle the ROS 2 initialization, spinning, and shutdown boilerplate.
    c) To define the message types.