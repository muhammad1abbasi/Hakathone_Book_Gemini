---
id: lesson-01-installation
title: "Lesson 1: Installing and Running Gazebo"
sidebar_position: 1
description: Your first steps into the Gazebo simulator, learning to create worlds and spawn simple objects.
---

### Lesson Objective
To install Gazebo and its ROS 2 integration packages, and to launch a pre-existing simulation world.

### Prerequisites
- Ubuntu 22.04 with ROS 2 Humble installed.

### Concept Explanation
**Gazebo** is a 3D robotics simulator. Think of it as a video game engine, but built specifically for robotics. It includes a high-performance physics engine (so things fall and collide realistically) and the ability to simulate a wide variety of sensors.

While Gazebo is a standalone program, its real power comes from its integration with ROS 2. A special set of packages, `gazebo_ros_pkgs`, provides plugins and tools that allow Gazebo to communicate seamlessly with our ROS 2 nodes. For example, a plugin can read the state of a simulated joint and publish it on a ROS 2 topic, and another plugin can listen for ROS 2 messages to control the lights in the simulated world.

The standard way to run Gazebo with ROS 2 is using a **launch file**. The launch file will start the Gazebo simulator and load a specific "world" file. A **world file** (with a `.world` or `.sdf` extension) is an XML-based file that describes everything in the simulation: the lighting, the physics properties (like gravity), and all the objects and robots.

### Real-World Analogy
This is like running a new video game for the first time.
- **Installing Gazebo** is like installing the game engine (e.g., Unreal Engine or Unity).
- **Installing `gazebo_ros_pkgs`** is like installing a mod that allows the game to communicate with external programs.
- The **`.world` file** is the specific game level or map file you are loading.
- **Running the launch file** is like double-clicking the game's icon, which boots up the engine and loads the specific level you want to play.

### Hands-On Task
**Task**: Install Gazebo and launch a demo world.

1.  **Install Gazebo**: The `gazebo_ros_pkgs` are not installed by default with the main ROS 2 desktop installation. You need to install them separately.
    ```bash
    sudo apt-get update
    sudo apt-get install ros-humble-gazebo-ros-pkgs
    ```
2.  **Explore Available Worlds**: Gazebo comes with several example worlds. You can find them in the Gazebo installation directory.
3.  **Launch a World**: We will launch the "Cafe" world, which is a simple environment with tables and chairs.
    ```bash
    # Don't forget to source ROS 2 in every new terminal!
    source /opt/ros/humble/setup.bash
    gazebo --verbose /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_cafe_world.world
    ```
4.  **Interact with Gazebo**:
    - The Gazebo client GUI will open.
    - **Navigate**:
        - **Left-click and drag**: Rotate the camera.
        - **Right-click and drag**: Zoom in and out.
        - **Middle-click (scroll wheel) and drag**: Pan the camera.
    - **Inspect**: On the left panel, you can see a tree view of all the models in the world.
    - **Add Objects**: On the top toolbar, there is an icon to add simple shapes (cubes, spheres, cylinders). Try adding a sphere and see it fall and roll on the ground.

### Python + ROS 2 Code Example
*No custom code is required for this lesson. We are only using command-line tools.*

### Common Mistakes & Debugging Tips
- **Mistake**: Gazebo runs very slowly or your computer's fans spin up loudly. Gazebo is a demanding 3D application. If you are running on a laptop without a dedicated graphics card or in a virtual machine, performance may be poor.
- **Tip**: You can run Gazebo without the GUI to save resources. The physics will still be simulated in the background. This is called running in "headless" mode. You can do this with the `-s` flag: `gazebo -s <world_file>`. This is very common for automated testing.
- **Error**: `gazebo: command not found`. This means the Gazebo package is not installed correctly or you forgot to source your ROS 2 `setup.bash` file.

### Mini Assessment
1.  What is Gazebo?
    a) A text editor for Python.
    b) A 3D robotics simulator with a physics engine.
    c) A ROS 2 message type.
2.  What is the purpose of the `gazebo_ros_pkgs`?
    a) They provide example robot models.
    b) They provide plugins to connect Gazebo with ROS 2.
    c) They are a different simulator.
3.  What is a `.world` file in Gazebo?
    a) A Python script for controlling a robot.
    b) An XML-based file that describes everything in a simulation scene.
    c) A log file of simulation events.
4.  In the Gazebo GUI, how do you rotate the camera view?
    a) Right-click and drag.
    b) Middle-click and drag.
    c) Left-click and drag.
5.  What does it mean to run Gazebo in "headless" mode?
    a) To run it without a robot model.
    b) To run the physics simulation server without the graphical client.
    c) To run it with a black background.
