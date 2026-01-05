---
id: lesson-04-testing-models
title: "Lesson 4: Visualizing URDFs with RViz"
sidebar_position: 4
description: Learn to use RViz and the robot_state_publisher to visualize your URDF model and test its joints.
---

### Lesson Objective
To use RViz, `robot_state_publisher`, and `joint_state_publisher_gui` to create a live, 3D visualization of your URDF model.

### Prerequisites
- Completion of the previous lessons in this chapter.
- A URDF file (`simple_humanoid.urdf`) in a ROS 2 package.

### Concept Explanation
Writing a URDF is only half the battle; you need to see it to know if it's correct. The primary tool for this is **RViz**, the ROS 2 visualizer. However, RViz doesn't know how to read a URDF file directly. It only knows how to display **transforms** (coordinate frames).

This is where two essential nodes come in:
1.  **`robot_state_publisher`**: This node reads your URDF file, finds the current state of all the joints (their angles), and then calculates the 3D position and orientation (the **transform**) of every link on the robot. It publishes these transforms to the `/tf` and `/tf_static` topics. RViz listens to these topics to draw the robot.

2.  **`joint_state_publisher_gui`**: But how does `robot_state_publisher` know the current angle of each joint? It listens to the `/joint_states` topic. The `joint_state_publisher_gui` provides a simple GUI with sliders for each non-fixed joint in your URDF. By moving the sliders, you publish `JointState` messages, which tells the `robot_state_publisher` to update the robot's pose, which RViz then displays.

This creates a pipeline:
`joint_state_publisher_gui` -> `/joint_states` topic -> `robot_state_publisher` -> `/tf` topic -> **RViz**

### Real-World Analogy
This pipeline is like digital puppetry.
- Your **URDF** is the puppet's design.
- The **`joint_state_publisher_gui`** is the puppeteer's controller, with sliders for each joint (elbow, shoulder, etc.).
- The **`robot_state_publisher`** is the physics engine that calculates the puppet's pose based on the puppeteer's commands. It figures out where the hand is based on the elbow and shoulder angles.
- **RViz** is the screen that displays the final rendered puppet.

### Hands-On Task
**Task**: Visualize your `simple_humanoid.urdf` and control it with a GUI.

1.  **Create a Launch File**: Bringing up all these nodes manually is tedious. A **launch file** is a script that starts multiple nodes with the correct configurations. In your `simple_robot_description` package, create a directory named `launch`. Inside it, create a file `display.launch.py`.
2.  **Write the Launch File**: Add the Python code below to your `display.launch.py`. This file automates finding your URDF and starting all three nodes.
3.  **Build the Package**: You need to build your package so ROS 2 can find your launch file and URDF.
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select simple_robot_description
    ```
4.  **Run the Launch File**:
    - Source your workspace: `source install/setup.bash`.
    - Run the launch file:
      ```bash
      ros2 launch simple_robot_description display.launch.py
      ```
5.  **Observe**: Two windows will open: RViz and the joint state publisher GUI.
    - In RViz, you'll see a blank screen at first. You need to configure it:
        - Change the "Fixed Frame" in the "Global Options" to `torso`.
        - Click "Add" -> "By topic" -> `/robot_description` -> `RobotModel`. Your robot should appear!
        - Click "Add" -> "By topic" -> `/tf` -> `TF`. You will see the coordinate frames.
    - In the joint state publisher GUI, move the slider for the `left_shoulder_joint`. You will see the arm move in real-time in RViz!

### Python + ROS 2 Code Example
```python
# display.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_path = get_package_share_directory('simple_robot_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'simple_humanoid.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[urdf_path]),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'urdf.rviz')]),
    ])
```
*(Note: For this to work perfectly, you would also need to create an RViz configuration file at `simple_robot_description/rviz/urdf.rviz`. For now, you can configure it manually as described in the task.)*

### Common Mistakes & Debugging Tips
- **Mistake**: The robot doesn't appear in RViz, or all links are piled at the origin. This usually means the `robot_state_publisher` isn't running or can't find the URDF file. Check the terminal where you ran the launch file for errors.
- **Mistake**: The robot appears, but the GUI sliders do nothing. This means there is a problem with the `/joint_states` topic. Use `ros2 topic echo /joint_states` to see if the GUI is publishing messages as you move the sliders.
- **Tip**: Save your RViz configuration! Once you have it set up to display your robot correctly, go to "File" -> "Save Config As" and save it inside your package. Then, you can have your launch file automatically load this config using the `-d` argument for RViz.

### Mini Assessment
1.  What is the main purpose of RViz?
    a) To write Python code.
    b) To provide a GUI for visualizing robot data and transforms.
    c) To build ROS 2 packages.
2.  Which node is responsible for reading the URDF and calculating the 3D position of each link?
    a) `rviz2`
    b) `joint_state_publisher_gui`
    c) `robot_state_publisher`
3.  What is a ROS 2 "launch file" used for?
    a) Starting a single node with default settings.
    b) Writing C++ code.
    c) Starting multiple nodes with specific configurations at the same time.
4.  What data does the `joint_state_publisher_gui` publish?
    a) The robot's final position.
    b) Messages on the `/joint_states` topic indicating the current angle of each joint.
    c) The URDF file content.
5.  If your robot model looks correct in RViz, but the arm is detached and floating in space, what is the most likely cause?
    a) A syntax error in the `<link>` tag.
    b) An incorrect `<origin>` tag in the shoulder joint, placing it in the wrong position.
    c) A problem with your graphics card.