---
id: lesson-01-urdf-simulation-tags
title: "Lesson 1: Adding Collision and Inertia to URDF"
sidebar_position: 1
description: Enhance URDF models with collision geometries and inertial properties essential for realistic physics simulation.
---

### Lesson Objective
To add `<collision>` and `<inertial>` tags to our humanoid URDF to enable realistic physical interactions and movement in Gazebo.

### Prerequisites
- Completion of Module 1, Chapter 4 (URDF Design).
- Basic understanding of physics concepts like mass and center of gravity.

### Concept Explanation
When we first learned about URDF, we focused on `<visual>` tags to describe how our robot *looks*. For simulation, we also need to describe how our robot *interacts physically* with the world. This is done with two new tags:

1.  **`<collision>`**: This tag defines the shape of the link for physics interactions. It's what the simulator uses to detect when two objects hit each other. Often, the collision geometry is simpler than the visual geometry to save computational power (e.g., a complex visual mesh might be represented by a simple box for collisions).
    -   Must contain a `<geometry>` tag (e.g., `<box>`, `<cylinder>`, `<sphere>`).
    -   Can have an `<origin>` to offset the collision shape relative to the link's origin.

2.  **`<inertial>`**: This tag defines the mass properties of the link. Without it, the link is considered massless by the physics engine, and it won't respond to forces correctly.
    -   **`<mass>`**: The mass of the link in kilograms.
    -   **`<inertia>`**: A 3x3 inertia matrix that describes how hard it is to rotate the link around its principal axes. This is a complex topic, but for simple shapes, Gazebo can often calculate it if you provide a `mass`. For more complex shapes, you might use a CAD tool to compute this.
    -   **`<origin>`**: The center of mass of the link, relative to the link's origin.

By adding these tags, our robot transitions from a purely visual model to a true physical object within the simulator.

### Real-World Analogy
Imagine a movie set.
- The **`<visual>`** tags are like the detailed, painted facades of buildings. They look real, but if you try to lean on them, you'll fall through.
- The **`<collision>`** tags are like the hidden, structural framework behind the facade. It's often simpler (maybe just a solid wall instead of individual bricks), but it's what prevents you from walking through it.
- The **`<inertial>`** tags are like adding weights inside the props to make them feel heavy and realistic when an actor interacts with them.

### Hands-On Task
**Task**: Add collision and inertial properties to your `torso` and `head` links in `simple_humanoid.urdf`.

1.  **Open `simple_humanoid.urdf`**: Locate the file in your `simple_robot_description/urdf` directory.
2.  **Add `collision` and `inertial` to `torso`**: Inside the `<link name="torso">` tag, add the `<collision>` and `<inertial>` blocks from the example below.
3.  **Add `collision` and `inertial` to `head`**: Do the same for the `<link name="head">` tag.
4.  **Check the URDF**: Use `check_urdf simple_humanoid.urdf` to ensure there are no XML errors. The tool might not validate the physics properties, but it will confirm syntax.
5.  **Visualize in Gazebo**: (This step will be fully explored in the next lesson). Try launching your robot with these new properties. If you add a block on top of your robot's head in Gazebo, it should now rest on the head instead of falling through!

### Python + ROS 2 Code Example
```xml
<!-- Excerpt from simple_humanoid.urdf with collision and inertial tags -->
<robot name="simple_humanoid">

  <!-- Torso Link -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.5"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>

    <!-- NEW COLLISION TAG for Torso -->
    <collision>
      <geometry>
        <box size="0.2 0.3 0.5"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <!-- END NEW COLLISION TAG -->

    <!-- NEW INERTIAL TAG for Torso -->
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <!-- END NEW INERTIAL TAG -->

  </link>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>

    <!-- NEW COLLISION TAG for Head -->
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
    </collision>
    <!-- END NEW COLLISION TAG -->

    <!-- NEW INERTIAL TAG for Head -->
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.05"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <!-- END NEW INERTIAL TAG -->

    <!-- Sensor block from previous lesson can remain here -->
    <sensor name="head_camera_sensor" type="camera">
      <pose>0.05 0 0.05 0 0 0</pose>
      <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
              <width>1280</width>
              <height>720</height>
              <format>R8G8B8</format>
          </image>
          <clip>
              <near>0.1</near>
              <far>100</far>
          </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
    </sensor>

  </link>

  <!-- ... (neck joint, other links and joints) ... -->

</robot>
```

### Common Mistakes & Debugging Tips
- **Mistake**: Forgetting to add `<collision>` tags. Your robot will appear in Gazebo but will fall through any surfaces.
- **Mistake**: Forgetting to add `<inertial>` tags. Your robot will appear, but any forces applied to it (including gravity) will have no effect, or it will behave unnaturally.
- **Tip**: Keep collision geometries as simple as possible (`box`, `sphere`, `cylinder`). Complex meshes in collisions significantly slow down the simulation. You can use different geometries for visual and collision.
- **Tip**: The `<inertia>` values are difficult to calculate manually. For simple shapes, you can often find calculators online. For complex CAD models, software like SolidWorks or Fusion 360 can compute them for you.

### Mini Assessment
1.  What is the primary purpose of the `<collision>` tag in a URDF for simulation?
    a) To define the color of the robot.
    b) To define the shape used for physics interactions and contact detection.
    c) To define how the robot moves.
2.  Why is it often recommended to use simpler geometries for `<collision>` than for `<visual>`?
    a) Simpler shapes are easier to draw.
    b) To improve simulation performance.
    c) So the robot looks better.
3.  What properties does the `<inertial>` tag define?
    a) The robot's speed and acceleration.
    b) The link's mass and how it resists rotation.
    c) The stiffness of the joints.
4.  If a simulated robot falls through the floor in Gazebo, what is the most likely missing tag in its URDF?
    a) `<visual>`
    b) `<inertial>`
    c) `<collision>` (on both the robot and the floor)
5.  What value is typically set for the `ixx`, `iyy`, `izz` fields in the `<inertia>` tag for a simple symmetrical shape?
    a) 0.0
    b) A non-zero positive value
    c) -1.0
