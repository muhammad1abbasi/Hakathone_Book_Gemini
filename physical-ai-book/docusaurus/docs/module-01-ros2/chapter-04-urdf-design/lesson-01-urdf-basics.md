---
id: lesson-01-urdf-basics
title: "Lesson 1: URDF Basics - Links and Joints"
sidebar_position: 1
description: Learn the fundamental XML syntax of URDF, including how to define a robot's links and joints.
---

### Lesson Objective
To write a simple URDF file from scratch that defines two **links** connected by a **joint**.

### Prerequisites
- A basic understanding of XML syntax (tags and attributes).

### Concept Explanation
URDF (Unified Robot Description Format) is an XML format used to describe the physical structure of a robot. The two most important tags in URDF are `<link>` and `<joint>`.

1.  **`<link>`**: A link represents a physical part of the robot. It has properties like its shape (for visualization and collision), its mass, and its inertia. For now, think of it as a single, rigid body part. Every robot has at least one special link called the **base link**, which is the root of the robot's structure.

2.  **`<joint>`**: A joint connects two links together and defines how they can move relative to each other. A joint has a **parent** link and a **child** link. It also has a **type**, which determines the kind of motion allowed:
    - `fixed`: No motion allowed. The two links are rigidly connected.
    - `continuous`: The child link can rotate infinitely around a single axis relative to the parent (like a wheel).
    - `revolute`: The child link can rotate around an axis, but it has upper and lower limits (like an elbow joint).
    - `prismatic`: The child link can slide along an axis, with limits (like a piston).

Every URDF file is wrapped in a `<robot>` tag and given a name.

```xml
<robot name="my_robot">
  <link name="base_link">
    ...
  </link>
  <link name="wheel_link">
    ...
  </link>
  
  <joint name="base_to_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    ...
  </joint>
</robot>
```

### Real-World Analogy
Building a URDF is like describing your own body.
- Your **torso** is the `base_link`.
- Your **upper arm**, **forearm**, and **hand** are all separate `<link>` elements.
- The connection between your torso and upper arm is your **shoulder**, which is a `<joint>`. The connection between your upper arm and forearm is your **elbow**, another `<joint>`.
- Your shoulder joint allows for complex motion, while your elbow joint is simpler (mostly `revolute`). A `fixed` joint would be like if your finger bones were fused together.

### Hands-On Task
**Task**: Create a simple URDF for a single arm.

1.  **Create a Package**: Just like with messages, URDFs should live in a package.
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python simple_robot_description
    ```
2.  **Create Directories**: Inside the new package, create directories to hold the URDF.
    ```bash
    cd simple_robot_description
    mkdir urdf
    ```
3.  **Create URDF File**: Inside the `urdf` directory, create a file named `simple_arm.urdf` and add the XML code from the example below.
4.  **Check the URDF**: ROS provides a tool to parse a URDF file and check for errors.
    ```bash
    # In the urdf directory
    check_urdf simple_arm.urdf
    ```
    If it's successful, you'll see a description of the robot's structure. If not, it will report syntax errors.

### Python + ROS 2 Code Example
```xml
<!-- simple_arm.urdf -->
<robot name="simple_arm">

  <!-- The Base Link (like the shoulder mount) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
  </link>

  <!-- The Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>

  <!-- The Joint connecting them -->
  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```
*Note: We've added a `<visual>` tag inside the links. This tells visualization tools like RViz what shape the link should have. It is not used for physics simulation.*

### Common Mistakes & Debugging Tips
- **Mistake**: XML syntax errors. Forgetting to close a tag (`</link>`) or having a typo in a tag name. The `check_urdf` tool is your best way to catch these.
- **Mistake**: Creating a kinematic loop. The structure of links and joints must be a **tree**, with the `base_link` as the root. You cannot have a chain of joints that loops back on itself (e.g., Link A -> Link B -> Link C -> Link A).
- **Tip**: Keep your link and joint names simple and descriptive (e.g., `left_wheel_joint`, `gripper_finger_link`). This makes the structure much easier to understand.

### Mini Assessment
1.  What are the two most fundamental tags in a URDF file?
    a) `<robot>` and `<visual>`
    b) `<link>` and `<joint>`
    c) `<parent>` and `<child>`
2.  What is the purpose of a `<link>` tag?
    a) To define how two parts move.
    b) To represent a physical, rigid part of the robot.
    c) To specify the robot's color.
3.  What does a `fixed` joint type do?
    a) It allows infinite rotation.
    b) It allows sliding motion.
    c) It connects two links rigidly with no movement.
4.  Every URDF structure must be a...
    a) Loop
    b) Tree
    c) Straight line
5.  What ROS command can be used to check a URDF file for syntax errors?
    a) `ros2 urdf check`
    b) `check_urdf <filename>`
    c) `urdf_validate <filename>`