---
id: lesson-03-message-types
title: "Lesson 3: Creating Custom Message Types"
sidebar_position: 3
description: Define your own custom message, service, and action types for specialized communication needs.
---

### Lesson Objective
To define, build, and use a custom message (`.msg`) file for a ROS 2 topic.

### Prerequisites
- Completion of Lesson 2: "Creating a Simple Robot Controller".
- A ROS 2 workspace (`ros2_ws`) structure.

### Concept Explanation
While ROS 2 provides a rich set of standard message types (`std_msgs`, `geometry_msgs`, etc.), you will often need to send data that doesn't fit into any of these. For this, you can create your own **custom message types**.

A custom message is defined in a `.msg` file. This file is just a simple text file where each line declares a data type and a name. For example:

```
string name
int32 age
bool is_student
```

You can also define custom **services** (`.srv`) and **actions** (`.action`). A `.srv` file has a request section and a response section, separated by `---`. An `.action` file has goal, result, and feedback sections, each separated by `---`.

To use these custom types, you need to:
1.  **Create a new Package**: You can't just have loose `.msg` files. They must be part of a proper ROS 2 package.
2.  **Define the Files**: Create a directory (e.g., `msg/`) and add your `.msg` files.
3.  **Modify `package.xml`**: Declare that your package depends on `rosidl_default_generators` and `rosidl_default_runtime`.
4.  **Modify `CMakeLists.txt` or `setup.py`**: Add instructions to find and build your message files. For Python packages, this is done in `setup.py`.
5.  **Build the Package**: Use `colcon build` to generate the Python code for your new message types.
6.  **Use It**: After sourcing the new `install/setup.bash`, you can import and use your custom message just like a standard one.

### Real-World Analogy
Think of ordering a custom-built computer online.
- **Standard Message Types** are like the pre-configured models (e.g., "The Gamer," "The Office Worker"). They have standard parts.
- **Custom Message Types** are like using the "Build Your Own" configuration tool. You get to define exactly what fields you want: `string cpu_type`, `int32 ram_gb`, `string gpu_model`. You are creating a new data structure. The website's backend (the ROS 2 build system) then takes your definition and makes it a real, orderable product.

### Hands-On Task
**Task**: Create a package and a custom message to represent a person's information.

1.  **Create a Package**: Navigate to your `ros2_ws/src` directory and run the package creation command:
    ```bash
    ros2 pkg create --build-type ament_python my_interfaces
    ```
2.  **Create `msg` Directory**: Inside the new `my_interfaces` package, create a directory named `msg`.
    ```bash
    cd my_interfaces
    mkdir msg
    ```
3.  **Define the `.msg` File**: Inside `my_interfaces/msg`, create a file named `Person.msg` with the following content:
    ```
    string name
    int32 age
    ```
4.  **Edit `package.xml`**: Open `my_interfaces/package.xml`. Add these lines inside the `<export>` tag:
    ```xml
    <build_tool_depend>rosidl_default_generators</build_tool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
5.  **Build the Package**: Navigate back to the root of your workspace (`cd ~/ros2_ws`) and build.
    ```bash
    colcon build --packages-select my_interfaces
    ```
6.  **Use the Custom Message**: Now, create a publisher that uses your new message. Create a file `person_publisher.py` in `ros2_ws/src`.
7.  **Run It**:
    - `source install/setup.bash` in your workspace root.
    - Run the publisher: `python3 src/person_publisher.py`.
    - In another terminal, echo the topic: `ros2 topic echo /person`. You will see your custom message!

### Python + ROS 2 Code Example
```python
# person_publisher.py (in ros2_ws/src)

import rclpy
from rclpy.node import Node
# Import the custom message
from my_interfaces.msg import Person

class PersonPublisher(Node):
    def __init__(self):
        super().__init__('person_publisher')
        self.publisher_ = self.create_publisher(Person, 'person', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Person()
        msg.name = "John Doe"
        msg.age = 30
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Name="{msg.name}", Age={msg.age}')

def main(args=None):
    rclpy.init(args=args)
    person_publisher = PersonPublisher()
    rclpy.spin(person_publisher)
    person_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Forgetting to build the package with `colcon build` after creating or changing a `.msg` file. The Python code is not generated until you build.
- **Mistake**: Forgetting to source the **workspace's** `install/setup.bash` file. You need to source this new file, not just the main ROS 2 one, for your terminal to find your custom package.
- **Error**: `ModuleNotFoundError: No module named 'my_interfaces'`.
- **Solution**: This almost always means you either haven't built the package successfully or you forgot to source the local setup file (`source install/setup.bash`).

### Mini Assessment
1.  Where must `.msg` files be defined?
    a) In any directory.
    b) In a ROS 2 package, inside a `msg/` subdirectory.
    c) Directly in your Python script.
2.  What command is used to build ROS 2 packages?
    a) `ros2 build`
    b) `make`
    c) `colcon build`
3.  Which file is modified to declare dependencies needed for generating messages?
    a) `CMakeLists.txt`
    b) `package.xml`
    c) `requirements.txt`
4.  After building a new interface package, what must you do before you can `import` it in a Python script?
    a) Restart your computer.
    b) Source the workspace's `install/setup.bash` file.
    c) Run `pip install .`.
5.  How do you separate the request and response fields in a `.srv` file?
    a) With a comma `,`
    b) With three hyphens `---`
    c) With a new line.