---
id: chapter-02-ros2-core
title: "Chapter 2: ROS 2 Core Concepts"
sidebar_position: 2
description: A deep dive into the essential building blocks of ROS 2 communication—Nodes, Topics, Services, and Actions.
---

## Chapter 2: ROS 2 Core Concepts

### Summary
This chapter is a technical deep dive into the heart of ROS 2. We will dissect the four primary methods of communication that you will use in every robotics project. We'll move from theory to practice, writing simple Python code to demonstrate how these components work together to create a functional robotics system.

### Why This Chapter Matters
Nodes, Topics, Services, and Actions are the fundamental building blocks of any ROS 2 application. Understanding when and how to use each one is non-negotiable for a robotics developer. This chapter provides the practical, hands-on knowledge you will rely on for the rest of this book and in your career.

### Real Robotics Use-Cases
- **Sensor Data Streaming**: A LiDAR sensor publishes its scan data on a Topic for any part of the system to consume.
- **Triggering an Action**: A navigation system sends a request to a camera Service to get the latest image before making a decision.
- **Long-Running Tasks**: Commanding a robot arm to pick up an object is a perfect use-case for an Action, which provides continuous feedback and can be preempted.

### Skills Students Will Build
- Ability to write a simple ROS 2 Node in Python.
- Proficiency in creating publishers and subscribers to communicate over Topics.
- Skill in implementing and calling Services for request/response interactions.
- Understanding of how to use Actions for long-running, feedback-driven tasks.