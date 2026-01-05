---
id: module-02-simulation
title: "Module 2: The Digital Twin – Gazebo & Unity"
sidebar_position: 2
description: Learn to create and interact with a digital twin of your robot in high-fidelity physics simulators like Gazebo and Unity.
---

## Module 2: The Digital Twin – Gazebo & Unity

### Module Overview
In this module, we bring our digital robot blueprint (the URDF) to life. A **Digital Twin** is a virtual, physically-accurate representation of our real robot that lives inside a simulator. We will learn how to use professional-grade tools like Gazebo and Unity to simulate our humanoid robot, its sensors, and its interaction with the environment. This allows us to safely develop and test our AI in a virtual world before deploying it on expensive hardware.

### Why This Module Matters
Developing on physical hardware is slow, expensive, and risky. A single bug in your code could cause a real robot to fall and break. Simulation accelerates development by providing a fast, free, and safe environment for iteration. You can test new AI algorithms, train models on synthetic data, and validate behaviors hundreds of times faster than you could in the real world. Mastering simulation is a critical skill for any modern roboticist.

### Skills Students Will Build
- Creating and customizing simulation worlds in Gazebo.
- Spawning a URDF-based robot model into a simulation.
- Understanding and tuning physics properties like gravity and friction.
- Simulating sensor data for cameras, LiDAR, and IMUs.
- Integrating the Unity engine for high-fidelity visualization and human-robot interaction scenarios.
- Controlling simulated robot joints via ROS 2 messages.

### Simulation Tools Used
- **Gazebo**: A powerful, open-source 3D robotics simulator that is tightly integrated with ROS.
- **Unity**: A popular game engine used for high-quality graphics and advanced scenario building.
- **ROS 2**: The communication bridge between our control code and the simulator.
- **URDF**: The robot description format used by the simulators.
- **Sensors**: We will learn to simulate the data streams from the most common robotics sensors.