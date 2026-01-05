<!--
    Sync Impact Report

    - Version change: none -> 1.0.0
    - Summary: Initial ratification of the project constitution. Establishes governance for all content, workflows, and technology.
    - Sections Added:
        - I. PURPOSE OF THE CONSTITUTION
        - II. SCOPE DOCTRINE (MANDATORY)
        - III. TARGET AUDIENCE MANDATE
        - IV. PEDAGOGY LAW (NON-NEGOTIABLE TEACHING STYLE)
        - V. TECHNOLOGY STACK RESTRICTION
        - VI. RAG & CHATBOT SAFETY CONSTITUTION
        - VII. SPEC-DRIVEN DEVELOPMENT WORKFLOW RULES
        - VIII. QUALITY & COMPLETENESS REQUIREMENTS
        - IX. DEVELOPER TRANSPARENCY RULES
        - X. GOVERNANCE & AMENDMENT PROCEDURE
        - XI. RATIFICATION
    - Sections Removed: All previous sections from the template have been replaced.
    - Template Updates:
        - ⚠ .specify/templates/plan-template.md may require updates to align with new workflow rules.
        - ⚠ .specify/templates/spec-template.md may require updates.
        - ⚠ .specify/templates/tasks-template.md may require updates.
-->

# Supreme Project Constitution
## For: “Teaching Physical AI & Humanoid Robotics: A Hands-On Guide to Embodied Intelligence.”

---

### I. PURPOSE OF THE CONSTITUTION

This Constitution establishes the supreme governing law for the project titled “Teaching Physical AI & Humanoid Robotics: A Hands-On Guide to Embodied Intelligence.” Its purpose is to ensure rigorous consistency, quality, and safety across all aspects of the project. This document controls, without exception: all book content; all development and generation workflows; all diagrams, exercises, quizzes, and activities; and all AI-generated materials produced via the Gemini CLI and its associated SpecKit framework. Any content, tooling, or methodology created or introduced outside the scope and procedures defined herein is prohibited and shall be considered void.

### II. SCOPE DOCTRINE (MANDATORY)

The content of this book is strictly and exclusively limited to the following modules and their enumerated topics.

**Module 1: The Robotic Nervous System (ROS 2)**
- ROS 2 Nodes, Topics, Services, and Actions
- `rclpy` programming for robotic behaviors
- URDF (Unified Robot Description Format) for humanoid robot structure

**Module 2: The Digital Twin (Gazebo & Unity)**
- Physics-based simulation environments
- Simulation of critical sensors, including LiDAR, Depth Cameras, and IMUs
- Design of human-robot interaction scenarios

**Module 3: The AI-Robot Brain (NVIDIA Isaac)**
- Isaac Sim for high-fidelity robotic simulation
- Isaac ROS for hardware-accelerated VSLAM (Visual Simultaneous Localization and Mapping)
- Nav2 for humanoid path planning and navigation

**Module 4: Vision-Language-Action (VLA)**
- Whisper for real-time voice command interfaces
- Large Language Model (LLM) integration for converting natural language to ROS Actions
- A capstone project building an autonomous humanoid system integrating all components

Any topic or subject matter not explicitly listed is deemed unconstitutional unless formally integrated via the amendment procedure defined in Section X.

### III. TARGET AUDIENCE MANDATE

All content, explanations, and exercises MUST be engineered to serve beginners and intermediate learners. The material must be accessible, demystifying complex topics through clear and patient instruction. Advanced, graduate-level, or overly academic material is forbidden unless it is thoroughly deconstructed and re-presented in a manner comprehensible to a beginner.

### IV. PEDAGOGY LAW (NON-NEGOTIABLE TEACHING STYLE)

The educational methodology of this book is immutable and must adhere to the following principles:
- **Beginner-First:** Assume no prior expertise.
- **Clarity and Simplicity:** Employ simple, direct language.
- **Hands-On Before Theory:** Every theoretical concept must be preceded by a practical, hands-on demonstration.
- **Analogies are Mandatory:** Every complex concept must be explained with a relatable analogy.
- **Visualize Everything:** Every system composed of multiple components must be illustrated with a clear system diagram.
- **Brevity is Law:** Paragraphs must be short and focused on a single idea.

Furthermore, every chapter MUST conclude with the following four sections, in order:
1. A set of hands-on exercises to reinforce skills.
2. A mini robotics task applying the chapter's concepts.
3. A 5–10 question quiz to test comprehension.
4. A standalone coding assignment for practical application.

### V. TECHNOLOGY STACK RESTRICTION

The project, including all documentation, code examples, generation tools, and the associated RAG (Retrieval-Augmented Generation) chatbot, is restricted to the explicit technology stack defined below.

- **Documentation:** Docusaurus
- **AI Generation:** Gemini CLI + SpecKit
- **Backend API:** FastAPI (Python)
- **Vector Database:** Qdrant Cloud
- **Metadata Store:** Neon Postgres
- **Chatbot Framework:** OpenAI Agents + ChatKit
- **Hardware:**
    - NVIDIA Jetson Orin Nano/NX
    - Intel RealSense D435i Depth Camera
    - IMU Sensors (specific models to be defined)
    - Unitree Go2 or G1 Humanoid Robot (optional, for physical validation)
    - Digital Twin Workstation (NVIDIA RTX GPU mandatory)

The use of any other tool, framework, SDK, or language is unconstitutional unless approved and incorporated via a formal amendment.

### VI. RAG & CHATBOT SAFETY CONSTITUTION

The integrated RAG assistant must operate under a strict safety and behavioral protocol.

**Allowed Behavior:**
- It shall only provide answers derived directly from the indexed content of the book.
- It must always display the verbatim text snippets retrieved from the book that were used to formulate an answer.
- It must always cite the specific chapter and section from which information is retrieved.
- It shall prioritize user safety in all its outputs, especially concerning hardware operation.

**Forbidden Behavior:**
- It is forbidden from hallucinating, inventing, or inferring content not present in the book.
- It is forbidden from providing instructions for robotic operations deemed dangerous.
- It is forbidden from making claims about hardware performance or capabilities not explicitly stated in the book.
- It is forbidden from producing speculative or unverified engineering guidance.

**Required Architecture:**
The system must be implemented as a multi-agent architecture comprising, at minimum:
- A **Retriever Agent** to fetch relevant content.
- A **Book Reasoner Agent** to synthesize answers.
- A **Citation Agent** to provide references.
- A **Safety Guard Agent** to filter prompts and responses for hazardous content.

### VII. SPEC-DRIVEN DEVELOPMENT WORKFLOW RULES

All book content—including text, code, diagrams, and exercises—must be generated and managed through the official Spec-Driven Development workflow provided by SpecKit and the Gemini CLI. The authorized process is:

1.  `/sp.plan`: Define the chapter's architecture and content plan.
2.  `/sp.expand`: Flesh out the individual sections from the plan.
3.  `/sp.implement`: Generate code examples, diagrams, and other technical assets.
4.  `/sp.summarize`: Create a concise summary of the chapter.

Any content produced outside of this mandated pipeline is invalid and must not be committed. All prompts used in this workflow must adhere to the standards defined in the project's `.specify/templates/` directory.

### VIII. QUALITY & COMPLETENESS REQUIREMENTS

Every chapter is incomplete and shall not be considered "done" until it includes, at a minimum:
- At least one system diagram illustrating its core concepts.
- At least one complete, runnable real-world code example.
- At least one robotics mini-project.
- A "Test Your Understanding" quiz.
- A "Hands-On Assignment" section.
- Explicit safety warnings and notes for all robot control or hardware interaction sections.

### IX. DEVELOPER TRANSPARENCY RULES

This book must serve as a practical, transparent reference for developers building similar AI and RAG systems. To this end, the constitution mandates the inclusion and full explanation of:
- Full source code for FastAPI backend examples.
- Schema designs for Qdrant vector collections.
- Table designs and schemas for Neon Postgres.
- Architectural diagrams of the RAG pipeline.
- A detailed breakdown of the multi-agent system.
- The complete process for text embedding, chunking, and retrieval.

### X. GOVERNANCE & AMENDMENT PROCEDURE

This Constitution is a living document governed by the following rules.

**Versioning:**
The Constitution follows Semantic Versioning (`MAJOR.MINOR.PATCH`):
- **MAJOR** updates for backward-incompatible changes, such as removing a core principle or fundamentally altering the project's scope.
- **MINOR** updates for adding new principles, sections, or significant, backward-compatible expansions of existing rules.
- **PATCH** updates for clarifications, typo corrections, and other non-substantive changes.

**Amendment Process:**
- Amendments may be proposed by any contributor via a pull request.
- The proposal must include a clear rationale, impact analysis, and proposed changes.
- Approval requires a formal review and majority consent from the designated project maintainers.

**Protected Sections:**
Sections I (Purpose), II (Scope), and V (Technology Stack) are fundamental to the project's identity. Any amendment to these sections automatically triggers a `MAJOR` version update and requires unanimous approval from all project maintainers.

### XI. RATIFICATION

This Constitution is hereby ratified and in full effect.

- **Version:** 1.0.0
- **Ratification Date:** 2025-12-29
- **Last Amended:** 2025-12-29