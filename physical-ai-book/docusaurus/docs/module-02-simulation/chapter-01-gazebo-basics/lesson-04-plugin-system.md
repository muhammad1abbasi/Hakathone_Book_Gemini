---
id: lesson-04-plugin-system
title: "Lesson 4: The Gazebo Plugin System"
sidebar_position: 4
description: An introduction to Gazebo plugins and how they bridge the gap between the simulator and ROS 2.
---

### Lesson Objective
To understand the role of plugins in Gazebo and how to add a pre-existing plugin (like a differential drive controller) to a URDF to enable robot control.

### Prerequisites
- Completion of Lesson 3: "Spawning Robots".

### Concept Explanation
By itself, Gazebo is just a physics simulator. It doesn't inherently know anything about ROS 2 topics, services, or actions. The magic link between Gazebo and ROS 2 is the **Gazebo Plugin System**.

A **plugin** is a compiled piece of code (usually C++) that can be loaded by Gazebo to add new functionality. The `gazebo_ros_pkgs` that we installed provide a rich set of plugins for common robotics tasks:
- **Sensor Plugins**: These attach to a `<sensor>` tag in a URDF and generate simulated sensor data, which they then publish on a ROS 2 topic. (e.g., `gazebo_ros_camera`, `gazebo_ros_imu_sensor`).
- **Controller Plugins**: These plugins create an interface to control the robot's joints. A very common one is the `gazebo_ros_diff_drive`, which subscribes to a `/cmd_vel` topic (of type `Twist`) and automatically controls the wheel joints of a differential-drive robot to achieve the commanded velocity.
- **World Plugins**: These can control aspects of the entire simulation, like the lighting or physics.

You add a plugin to your URDF inside a `<gazebo>` tag. You specify the plugin's name and any parameters it needs, like the topic name to publish on or the names of the joints to control.

```xml
<gazebo reference="base_link">
  <plugin
    name="gazebo_ros_diff_drive"
    filename="libgazebo_ros_diff_drive.so">
    
    <!-- Parameters for the plugin -->
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <odometry_topic>odom</odometry_topic>
    ...
  </plugin>
</gazebo>
```

### Real-World Analogy
Gazebo plugins are like mods for a video game.
- The **base game** is Gazebo (physics, graphics).
- You want to add a new feature, like having the in-game cars be drivable by an external program.
- You download and install a **mod** (the plugin, e.g., `libgazebo_ros_diff_drive.so`).
- You edit a configuration file (the URDF) to tell the game which car model this mod should apply to and what settings to use.
Now, the mod "listens" for commands from your external program (ROS 2 messages) and applies the correct forces to the car's wheels inside the game's physics engine.

### Hands-On Task
**Task**: Add the differential drive plugin to a simple robot and control it using `/cmd_vel` messages.

1.  **Create a Drivable Robot URDF**: We need a robot with two wheels. Create a new URDF file named `diff_drive_robot.urdf` in your `simple_robot_description/urdf` directory. Copy the example code below. This URDF defines a chassis and two wheels connected by `continuous` joints. Critically, it also includes the `<plugin>` block.
2.  **Create a New Launch File**: Create a new launch file `spawn_diff_drive.launch.py`. This will be similar to our last launch file but will load the new URDF.
3.  **Build and Launch**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select simple_robot_description
    source install/setup.bash
    ros2 launch simple_robot_description spawn_diff_drive.launch.py
    ```
4.  **Drive the Robot**: Gazebo will launch and spawn the new robot. Now, open a new terminal and use `ros2 topic pub` to send a velocity command.
    - **Move forward**:
      ```bash
      ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
      ```
    - **Turn in place**:
      ```bash
      ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
      ```
5.  **Observe**: Your robot will move and turn in Gazebo in response to your ROS 2 commands! The plugin is doing all the work of converting your `Twist` message into wheel velocities.

### Python + ROS 2 Code Example
```xml
<!-- diff_drive_robot.urdf -->
<robot name="diff_drive_robot">
  <link name="base_footprint"/>

  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.3 0.1"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.5 0.3 0.1"/></geometry>
    </collision>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- ... (define left_wheel_link and right_wheel_link) ... -->
  <!-- ... (define left_wheel_joint and right_wheel_joint) ... -->
  
  <gazebo>
    <plugin
      name="gazebo_ros_diff_drive"
      filename="libgazebo_ros_diff_drive.so">

      <!-- Joints to control -->
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>

      <!-- Kinematics -->
      <wheel_separation>0.35</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>

      <!-- Odometry -->
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>

      <!-- Command topic -->
      <command_topic>cmd_vel</command_topic>
    </plugin>
  </gazebo>
</robot>
```
*(Note: This is a simplified URDF. A full version would include the wheel links and joints.)*

### Common Mistakes & Debugging Tips
- **Mistake**: The plugin doesn't load. The most common reason is a typo in the plugin `name` or `filename`. Check the Gazebo output in the terminal for errors about failing to load a library.
- **Mistake**: The robot flips over or acts erratically. This usually points to a problem with the physics properties. The joints, mass (`<inertial>` tags), and friction values need to be set realistically.
- **Tip**: To find available plugins and their parameters, you can look at the source code or documentation for `gazebo_ros_pkgs`. Many tutorials and robot models online also provide excellent examples of how to configure plugins.

### Mini Assessment
1.  What is the primary role of a Gazebo plugin?
    a) To define the visual look of a robot.
    b) To add functionality to the simulator and connect it to external systems like ROS 2.
    c) To define the physics properties of the world.
2.  Which ROS package provides a set of standard, pre-made plugins for ROS 2 integration?
    a) `rqt_gui`
    b) `urdf`
    c) `gazebo_ros_pkgs`
3.  To control a two-wheeled robot with `/cmd_vel` messages, which plugin would be most appropriate?
    a) `gazebo_ros_camera`
    b) `gazebo_ros_diff_drive`
    c) `gazebo_ros_imu_sensor`
4.  Where in the URDF do you typically define a Gazebo plugin?
    a) Inside a special `<gazebo>` tag.
    b) At the top of the file.
    c) Inside the `<joint>` tag it controls.
5.  If you add a camera plugin to your URDF, what do you expect it to do?
    a) Change the color of the robot.
    b) Publish simulated camera images to a ROS 2 topic.
    c) Make the simulation run faster.
