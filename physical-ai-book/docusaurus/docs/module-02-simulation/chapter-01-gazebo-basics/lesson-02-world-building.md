---
id: lesson-02-world-building
title: "Lesson 2: Building Your First World"
sidebar_position: 2
description: Learn to write a simple Gazebo world file from scratch using the SDF format.
---

### Lesson Objective
To create a custom Gazebo world file that defines a ground plane, lighting, and a few static objects.

### Prerequisites
- Completion of Lesson 1: "Installing and Running Gazebo".

### Concept Explanation
Gazebo worlds are defined using the **Simulation Description Format (SDF)**. SDF is an XML-based format that describes everything about a simulation: robots, lights, sensors, and the environment itself. A file containing this information is typically saved with a `.world` extension.

The basic structure of a `.world` file is:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Physics and scene settings go here -->
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Other models (objects) go here -->

  </world>
</sdf>
```
**Key SDF Tags:**
- **`<world>`**: The root element that contains everything.
- **`<physics>`**: Defines the physics engine properties, most importantly gravity.
- **`<scene>`**: Defines the visual properties of the world, like background color and shadows.
- **`<include>`**: A very useful tag that lets you include pre-existing models. Gazebo comes with built-in models like `ground_plane` and `sun`. Models can also be downloaded from the Gazebo Fuel online repository.
- **`<model>`**: Defines a new object in the world. A model is a collection of links, joints, and sensors. For simple static objects, it's often just one link.
- **`<link>`**: A physical part of a model.
- **`<visual>`**: The visual appearance of a link (what it looks like).
- **`<collision>`**: The physical shape of a link used by the physics engine. This can be (and often should be) simpler than the visual shape.
- **`<pose>`**: A crucial tag used everywhere. It defines the position (x, y, z) and orientation (roll, pitch, yaw) of an element relative to its parent.

### Real-World Analogy
Building a `.world` file is like being a level designer for a video game or a set designer for a movie.
- You start with an empty stage (`<world>`).
- You define the overall lighting (`<include><uri>model://sun</uri></include>`).
- You lay down the floor (`<include><uri>model://ground_plane</uri></include>`).
- You then start placing props (`<model>`). For each prop, you define its position (`<pose>`), what it looks like (`<visual>`), and its physical shape (`<collision>`).

### Hands-On Task
**Task**: Create a simple world with a ground plane, a sun, and a box.

1.  **Create a `worlds` Directory**: In your `simple_robot_description` ROS 2 package, create a new directory named `worlds`.
2.  **Create World File**: Inside the `worlds` directory, create a file named `my_first_world.world`.
3.  **Write the SDF Code**: Copy the SDF code from the example below into your new file.
4.  **Launch the World**: Open a terminal and use the `gazebo` command to launch your custom world. Make sure to use the full path to your file.
    ```bash
    # Make sure you are in your ros2_ws
    source /opt/ros/humble/setup.bash
    gazebo --verbose src/simple_robot_description/worlds/my_first_world.world
    ```
5.  **Observe**: Gazebo will launch and you should see a ground plane with a single blue box sitting on it, illuminated by a sun.

### Python + ROS 2 Code Example
```xml
<!-- my_first_world.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_first_world">
    
    <!-- Set default physics engine and gravity -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Include the ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include the sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple static box model -->
    <model name="my_box">
      <static>true</static>
      <pose>1.0 2.0 0.5 0 0 0</pose>
      <link name="box_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

  </world>
</sdf>
```

### Common Mistakes & Debugging Tips
- **Mistake**: XML syntax errors. Like with URDF, a missing closing tag can break the whole file. Run Gazebo with the `--verbose` flag to see parsing errors.
- **Mistake**: Objects appear halfway through the floor. The `<pose>` tag defines the position of the object's **origin**. If a 1-meter-tall box has its origin at its center, its origin must be placed at `z=0.5` for it to sit on top of the ground plane at `z=0`.
- **Tip**: Keep your `<visual>` and `<collision>` geometries simple. For a table, the visual might have four detailed legs and a top, but the collision can just be five simple boxes. Complex collision meshes slow down the physics engine significantly.

### Mini Assessment
1.  What format is used to define Gazebo worlds?
    a) URDF
    b) XML
    c) SDF
2.  In an SDF file, what does the `<include>` tag do?
    a) It includes another `.world` file.
    b) It imports a pre-existing model from a URI.
    c) It adds a new plugin.
3.  What is the difference between the `<visual>` and `<collision>` tags?
    a) There is no difference.
    b) `<visual>` is for looks, `<collision>` is for the physics engine.
    c) `<visual>` is for robots, `<collision>` is for static objects.
4.  If a sphere model has a radius of 0.5 meters, what should the z-value of its `<pose>` be to have it rest on the ground at z=0?
    a) 0.0
    b) 0.5
    c) 0.25
5.  What does the `<gravity>` tag inside the `<physics>` block define?
    a) The direction and magnitude of gravity in the simulation.
    b) The importance of the world file.
    c) The default color of objects.
