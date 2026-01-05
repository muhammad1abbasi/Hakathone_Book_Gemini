---
id: lesson-02-humanoid-joints
title: "Lesson 2: Describing Humanoid Joints"
sidebar_position: 2
description: Defining a kinematic tree for a humanoid robot with revolute joints for arms and legs.
---

### Lesson Objective
To expand our simple URDF to represent a basic humanoid robot, including a torso, head, and two arms, using a combination of `fixed` and `revolute` joints.

### Prerequisites
- Completion of Lesson 1: "URDF Basics".

### Concept Explanation
A humanoid robot is a complex tree of links and joints. The key is to build it up piece by piece.
- **The Root**: We start with a `base_link`, which is often the robot's torso.
- **Building Up**: We add a `head_link` connected to the torso with a `neck_joint`. This might be a `revolute` joint to allow the head to pan left and right.
- **Building Out**: We add `upper_arm_link`s connected to the torso with `shoulder_joint`s. These are complex joints, but we can simplify them as simple `revolute` joints for now.
- **Chaining**: We then connect a `forearm_link` to the `upper_arm_link` with an `elbow_joint`. The `upper_arm_link` is the **parent**, and the `forearm_link` is the **child**.

This creates a **kinematic chain**. The position of the hand depends on the angle of the elbow, which in turn depends on the angle of the shoulder. The URDF defines this entire structure.

A critical tag for joints is `<origin>`. This tag specifies the offset of the child link from the parent link's origin. It's how you position the joint correctly in 3D space.

### Real-World Analogy
It's like assembling a toy action figure.
1. You start with the torso piece (`base_link`).
2. You snap the head onto the neck peg. The peg's location is the joint's `<origin>`, and the snapping action creates the `neck_joint`.
3. You snap the left upper arm into the left shoulder socket. This creates the `left_shoulder_joint`.
4. You then snap the left forearm onto the left elbow peg of the upper arm piece. This creates the `left_elbow_joint`.

You are physically building the kinematic tree.

### Hands-On Task
**Task**: Extend the `simple_arm.urdf` to include a head.

1.  **Open the File**: Open your `simple_arm.urdf` from the previous lesson.
2.  **Add a Head Link**: Add a new `<link>` element named `head_link`. Give it a `<visual>` shape, like a sphere.
3.  **Add a Neck Joint**: Add a new `<joint>` element named `neck_joint`.
    - Its type should be `fixed` for now.
    - The `<parent>` should be `base_link`.
    - The `<child>` should be `head_link`.
    - Define an `<origin>` tag to place the head on top of the base. For example, `xyz="0 0 0.1"`.
4.  **Check the URDF**: Run `check_urdf simple_arm.urdf` again to ensure your syntax is correct.

### Python + ROS 2 Code Example
```xml
<!-- simple_humanoid.urdf -->
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
  </link>

  <!-- Neck Joint (Fixed) -->
  <joint name="neck_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Left Upper Arm Link -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 1.57 0"/>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Left Shoulder Joint (Revolute) -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.2 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```

### Common Mistakes & Debugging Tips
- **Mistake**: Incorrectly setting the `<origin>` tag for a joint. This can cause parts of your robot to appear in the wrong place or rotate around the wrong point.
- **Tip**: Visualize! It is very difficult to get a URDF right without seeing it. In the next lesson, we'll learn how to use RViz to visualize our URDF. This is the most important debugging tool for URDF development. If a link is in the wrong place, tweak the `<origin>` tag of its joint and reload the model.
- **Mistake**: Making joint axes incorrect. `xyz="0 0 1"` means rotate around the Z axis. `xyz="0 1 0"` means rotate around the Y axis.

### Mini Assessment
1.  In a humanoid URDF, which link is typically the root of the kinematic tree?
    a) The head
    b) The hand
    c) The torso (or `base_link`)
2.  What is a "kinematic chain"?
    a) A physical chain used to move the robot.
    b) A sequence of links and joints where the position of one depends on the one before it.
    c) A list of all the sensors on the robot.
3.  What is the purpose of the `<origin>` tag inside a `<joint>`?
    a) It specifies the child link's position and orientation relative to the parent link.
    b) It defines the robot's starting position in the world.
    c) It sets the color of the joint.
4.  If you connect `link_B` to `link_A` with a joint, which is the parent?
    a) `link_B`
    b) `link_A`
    c) They are both parents.
5.  You want to create a joint for a robot's knee, which can only bend within a certain range. Which joint type would you use?
    a) `continuous`
    b) `fixed`
    c) `revolute`