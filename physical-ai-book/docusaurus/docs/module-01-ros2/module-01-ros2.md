---
id: module-01-ros2
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_position: 1
description: An introduction to the core concepts of the Robot Operating System 2 (ROS 2), the foundational communication layer for modern robotics.
---

## Module 1: The Robotic Nervous System (ROS 2)

### Module Summary

Welcome to the foundational module of your journey into physical AI. In this module, we will build the "nervous system" for our robot using the Robot Operating System 2 (ROS 2). You will learn how different parts of a robot's software can communicate with each other in a standardized, powerful, and scalable way. Think of ROS 2 as the universal language that allows the "brain" (our AI) to talk to the "muscles" (motors and sensors).

### Why This Module Matters

A robot is a complex system of sensors, actuators, and decision-making algorithms. Without a robust communication backbone, these components cannot work together. ROS 2 provides that backbone. Mastering it is the first and most critical step to building any serious robotics application. It's the difference between a collection of disconnected parts and a fully integrated, intelligent agent.

### Skills Students Will Build

- Understanding of the publish/subscribe messaging pattern.
- Ability to create and manage ROS 2 nodes.
- Proficiency in using ROS 2 Topics, Services, and Actions for different communication needs.
- Skill in writing basic Python scripts that interact with the ROS 2 graph using the `rclpy` library.
- Foundational knowledge of how to represent a robot's physical structure using URDF.

### Tools & Tech Stack Used

- **ROS 2 (Humble Hawksbill)**: The core robotics middleware.
- **Ubuntu 22.04**: The primary operating system for ROS 2 development.
- **Python 3.10+**: The programming language for our ROS 2 nodes.
- **rclpy**: The official Python client library for ROS 2.
- **Visual Studio Code**: Our recommended code editor with ROS extensions.