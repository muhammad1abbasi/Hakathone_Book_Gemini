---
id: chapter-02-urdf-modeling
title: "Chapter 2: Simulation-Ready URDFs"
sidebar_position: 2
description: Enhancing our URDFs with simulation-specific tags for physics, materials, and sensor plugins.
---

## Chapter 2: Simulation-Ready URDFs

### Summary
In this chapter, we level up our URDFs to make them ready for high-fidelity simulation. We'll move beyond simple `<visual>` tags and add `<collision>` and `<inertial>` properties to give our robot a physical presence. We will also explore how to add Gazebo-specific tags, including material properties (like friction) and plugins that connect our URDF to ROS 2 topics.

### Why This Chapter Matters
A URDF for visualization is simple, but a URDF for simulation needs to describe how the robot interacts physically with its world. Without proper collision and inertia tags, your simulated robot would be a "ghost," passing through walls and having no mass. This chapter covers the critical additions needed to turn a visual model into a true, physically-simulated digital twin.

### Real Robotics Use-Cases
- **Grasping Simulation**: A robot arm URDF needs accurate collision geometry on its gripper fingers to simulate picking up an object.
- **Walking & Balancing**: A humanoid robot needs correct inertial properties (mass, center of gravity) for each link to simulate walking without falling over.
- **Realistic Rendering**: Using Gazebo's material properties to make a robot model look metallic or rubbery in the simulator.

### Skills Students Will Build
- Adding `<collision>` tags to define a robot's physical boundaries.
- Adding `<inertial>` tags to define a link's mass and rotational inertia.
- Understanding the difference between `<visual>` and `<collision>` geometries.
- Adding `<gazebo>`-specific tags for materials and plugins.
- Using the `ros2_control` tag to set up joint controllers for the simulator.
