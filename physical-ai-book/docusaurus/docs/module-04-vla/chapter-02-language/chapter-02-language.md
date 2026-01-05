---
id: chapter-02-language
title: "Chapter 2: LLMs for Robot Task Planning"
sidebar_position: 2
description: Leveraging Large Language Models (LLMs) to interpret natural language commands and generate robot-executable task plans.
---

## Chapter 2: LLMs for Robot Task Planning

### Summary
This chapter bridges the gap between human language and robot action. We will explore how Large Language Models (LLMs) can interpret natural language commands from users, understand their intent, and translate that into a structured, robot-executable task plan. You will learn about effective prompt engineering strategies to guide LLMs towards generating sequences of ROS 2 actions or service calls, transforming vague instructions into concrete steps a robot can understand.

### Why This Chapter Matters
LLMs represent a paradigm shift in human-robot interaction. Instead of pre-programming every possible robot behavior, we can now empower robots to understand and adapt to a vast array of natural language commands. This significantly reduces the complexity of robot programming and allows for more flexible and intuitive control, unlocking new possibilities for robot autonomy.

### Real-World Robotics Use-Cases
- **Domestic Robots**: Telling a robot, "Please set the table," which it breaks down into "go to the cupboard," "get plates," "go to the table," "place plates."
- **Industrial Assistants**: A technician instructing a robot arm, "Pick up the blue wrench and hand it to me," where the robot identifies the wrench and plans the grasp and delivery sequence.
- **Exploration Rovers**: Commanding a rover, "Investigate the rocky outcrop on the left," and the rover autonomously plans a safe path and executes scientific observation.

### Skills Students Will Build
- Understanding how LLMs can be used for task decomposition and planning in robotics.
- Designing effective prompts for LLMs to extract intent and generate structured robot plans.
- Translating high-level natural language goals into sequences of ROS 2 primitives (topics, services, actions).
- Implementing a ROS 2 node that communicates with an LLM API to get task plans.
- Handling ambiguity and error correction in LLM-generated plans.
