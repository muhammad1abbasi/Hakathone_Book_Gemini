---
id: lesson-03-human-vs-robot-agency
title: "Lesson 3: Human vs. Robot Agency"
sidebar_position: 3
description: Understanding the spectrum of autonomy in robots, from direct human control to full self-directed agency.
---

### Lesson Objective
To differentiate between the levels of robot autonomy (direct control, shared autonomy, full autonomy) and understand the engineering and ethical considerations of each.

### Prerequisites
- Completion of Lesson 2: "Embodied Intelligence".

### Concept Explanation
**Agency** refers to the capacity of an agent to act independently and make its own choices. For robots, agency exists on a spectrum:

1.  **Direct Control (Teleoperation)**: A human is in complete control. The robot is like a puppet. The human uses a joystick or controller to command the robot's every move. The robot has no agency.
    *   *Example*: A bomb-disposal robot controlled by an operator watching a screen.

2.  **Shared Autonomy (Human-on-the-Loop)**: The human and the robot work as a team. The human might give a high-level command like "go to the kitchen," and the robot will autonomously navigate, avoiding obstacles. The human can intervene at any time.
    *   *Example*: A surgical robot where the surgeon guides the instruments, but the robot stabilizes the motion and prevents accidental slips.

3.  **Full Autonomy**: The robot makes its own decisions to achieve a goal. It perceives the environment, plans its actions, and executes them without human intervention.
    *   *Example*: A Mars rover navigating the Martian surface, making decisions on its own for hours at a time due to communication delays.

### Real-World Analogy
Think of driving a car.
- **Direct Control** is like a go-kart where you control the steering and speed directly.
- **Shared Autonomy** is like using cruise control. You set a high-level goal ("maintain 65 mph"), and the car handles the throttle, but you are still steering and can take over at any moment. Modern lane-keeping assist is another great example.
- **Full Autonomy** is the goal of a self-driving car, which can navigate from point A to point B with no human input at all.

### Hands-On Task
**Task**: Classify different robotic systems based on their level of agency.
1.  Open a text editor.
2.  For each system below, label it as "Direct Control," "Shared Autonomy," or "Full Autonomy" and write a one-sentence justification.
    *   A remote-controlled toy car.
    *   A factory robot arm that picks up parts and places them in a box, but a human can press an emergency stop button.
    *   A drone that is given a GPS coordinate and flies there by itself, avoiding birds and buildings along the way.

### Python + ROS 2 Code Example
*No code is required for this conceptual lesson. We will begin coding in Chapter 2.*

### Common Mistakes & Debugging Tips
- **Mistake**: Assuming "autonomy" is an all-or-nothing concept. Most useful robots today operate in the "Shared Autonomy" space.
- **Tip**: When building a robot, always start with Direct Control. It's the simplest and safest way to test your hardware before you start giving it a mind of its own.

### Mini Assessment
1.  A robot controlled entirely by a person with a joystick is an example of:
    a) Full Autonomy
    b) Shared Autonomy
    c) Direct Control (Teleoperation)
2.  What is the defining characteristic of "Shared Autonomy"?
    a) The robot has no sensors.
    b) The human and robot work together as a team.
    c) The robot operates without any possibility of human intervention.
3.  Why does the Mars rover need a high degree of autonomy?
    a) The surface is very smooth.
    b) There is a significant time delay in communications with Earth.
    c) It has no scientific instruments.
4.  Lane-keeping assist in a modern car is an analogy for:
    a) Full Autonomy
    b) Shared Autonomy
    c) Direct Control
5.  What is the safest way to begin testing a new robot's hardware?
    a) Activate its full autonomous mode immediately.
    b) Use Direct Control to test basic movements.
    c) Let it learn on its own through trial and error.