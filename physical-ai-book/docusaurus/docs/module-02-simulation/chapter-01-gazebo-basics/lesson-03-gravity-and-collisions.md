---
id: lesson-03-gravity-and-collisions
title: "Lesson 3: Spawning Robots"
sidebar_position: 3
description: Learn how to spawn a robot model from a URDF file into a running Gazebo simulation.
---

### Lesson Objective
To use a ROS 2 launch file to start Gazebo and spawn a robot described by a URDF file into the simulation at a specific location.

### Prerequisites
- Completion of Lesson 2: "Building Your First World".
- A URDF file (e.g., `simple_humanoid.urdf`) located in a ROS 2 package.

### Concept Explanation
We have a world file and a robot file (URDF), but how do we get the robot *into* the world? You don't typically add the robot directly to the `.world` file. Instead, you launch the world, and then you "spawn" the robot into the running simulation.

This is done using a node from the `gazebo_ros` package called `spawn_entity.py`. This script can be called from the command line, but it's much more convenient to manage it within a launch file.

The process in a launch file looks like this:
1.  **Launch Gazebo**: Start the Gazebo server (`gzserver`) and client (`gzclient`) and tell it which `.world` file to load.
2.  **Load the URDF**: Find the URDF file on the disk and load its content into a string.
3.  **Start `robot_state_publisher`**: This is the same node from the previous module. It reads the URDF and gets ready to publish the robot's transforms. It needs the URDF content to know the robot's structure.
4.  **Start the `spawn_entity` Node**: This node takes the URDF content and a "spawn name" and makes a service call to a Gazebo service (usually `/spawn_entity`) to add the robot to the simulation. You can also specify the initial x, y, z position and orientation.

This separation allows you to load any robot into any world, which is a very flexible and powerful workflow.

### Real-World Analogy
This is like playing a "sandbox" video game like Minecraft or Garry's Mod.
1.  You first **load the map** (launch Gazebo with a `.world` file). The world is empty initially.
2.  You then open a menu of items you can place in the world. You select a character model (your URDF).
3.  You use a special tool (the `spawn_entity` node) to place that character into the map at the location you choose.
The character model and the map were separate files, and you combined them at runtime.

### Hands-On Task
**Task**: Create a launch file that opens your custom world and spawns your simple humanoid robot into it.

1.  **Locate your URDF**: Make sure your `simple_humanoid.urdf` from Module 1 is inside your `simple_robot_description/urdf` directory.
2.  **Locate your World**: Make sure your `my_first_world.world` from the last lesson is in `simple_robot_description/worlds`.
3.  **Create a Launch File**: In your `simple_robot_description/launch` directory, create a new file named `spawn_robot.launch.py`.
4.  **Write the Launch Code**: Copy the Python launch file code from the example below into your new file. Study the different sections.
5.  **Build and Launch**:
    - Build your workspace to make sure ROS 2 can find all your files:
      ```bash
      cd ~/ros2_ws
      colcon build --packages-select simple_robot_description
      ```
    - Source the workspace and run the launch file:
      ```bash
      source install/setup.bash
      ros2 launch simple_robot_description spawn_robot.launch.py
      ```
6.  **Observe**: Gazebo will launch, load your world with the blue box, and after a moment, your `simple_humanoid` robot model will appear at the origin.

### Python + ROS 2 Code Example
```python
# spawn_robot.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('simple_robot_description')

    # Start Gazebo server and client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_robot_description, 'worlds', 'my_first_world.world')}.items()
    )

    # Load the URDF file
    urdf_path = os.path.join(pkg_robot_description, 'urdf', 'simple_humanoid.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Start robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Start the spawner node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_humanoid', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '1.0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

### Common Mistakes & Debugging Tips
- **Mistake**: The robot spawns but immediately falls through the floor and disappears. This means the robot's `<collision>` tags are missing or incorrect, or the ground plane has no collision properties.
- **Mistake**: The launch file fails with an error like "file not found". This usually means the path to your URDF or world file is incorrect. Use `os.path.join` and `get_package_share_directory` to build robust paths, as shown in the example.
- **Tip**: You can change the spawn location by changing the `-x`, `-y`, and `-z` arguments in the `spawn_entity` node. Try changing `z` to `5.0` and watch the robot fall from the sky!

### Mini Assessment
1.  How are robots typically added to a running Gazebo simulation?
    a) By editing the `.world` file directly.
    b) By using the `spawn_entity.py` script.
    c) By dragging and dropping the URDF file into the window.
2.  What information does the `spawn_entity.py` node need?
    a) The robot's name (`-entity`) and its URDF description (`-topic`).
    b) The name of the world file.
    c) The IP address of the computer running Gazebo.
3.  Why is it better to use a launch file for this process?
    a) It's the only way to start Gazebo.
    b) It automates the multi-step process of starting Gazebo, `robot_state_publisher`, and the spawner.
    c) It makes the simulation run faster.
4.  If a robot spawns and falls through the ground, what is the most likely problem in its URDF?
    a) Missing `<visual>` tags.
    b) Missing or incorrect `<collision>` tags.
    c) The robot's name is wrong.
5.  What ROS 2 package provides the Gazebo plugins and spawner script?
    a) `rviz2`
    b) `gazebo_ros`
    c) `robot_state_publisher`
