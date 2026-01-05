---
id: 001
title: Generate Supreme Project Constitution
stage: constitution
date: 2025-12-29
surface: agent
model: gemini-pro
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "governance", "setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution

You are an AI-Driven Development Expert and a PhD-level book architect.  
Your task is to generate the Supreme Project Constitution for a technical book titled:

**“Teaching Physical AI & Humanoid Robotics: A Hands-On Guide to Embodied Intelligence.”**

This constitution must govern ALL content creation, workflows, structure, pedagogy, technology stack, RAG integrity, and SpecKit/Gemini generation rules for the entire project.

Your output MUST be written in a constitutional, enforceable, unambiguous, legal-style tone — NOT conversational.

Your constitution MUST include the following sections:

============================================================
I. PURPOSE OF THE CONSTITUTION  
Define why this constitution exists and what it controls:  
- ALL book content  
- ALL generation workflows  
- ALL diagrams, exercises, quizzes, activities  
- ALL AI-generated materials via Gemini CLI + SpecKit  
- Prohibition of content outside the approved scope  
============================================================

============================================================
II. SCOPE DOCTRINE (MANDATORY)  
The book MUST stay strictly limited to the following modules and topics:

**Module 1: The Robotic Nervous System (ROS 2)**
- ROS2 Nodes, Topics, Services, Actions  
- rclpy programming  
- URDF humanoid structure

**Module 2: The Digital Twin (Gazebo & Unity)**
- Physics simulation  
- Sensor simulation (LiDAR, Depth, IMU)  
- Human-robot interaction scenes

**Module 3: The AI-Robot Brain (NVIDIA Isaac)**
- Isaac Sim  
- Isaac ROS VSLAM  
- Nav2 humanoid path planning

**Module 4: Vision-Language-Action (VLA)**
- Whisper voice interface  
- LLM → ROS Action planning  
- Autonomous humanoid capstone system

All additional content MUST be considered unconstitutional unless explicitly added by amendment.
============================================================

============================================================
III. TARGET AUDIENCE MANDATE  
All teaching must serve:  
- Beginners  
- Intermediate learners  
NO advanced, graduate-level or overly academic content unless broken down for beginners.
============================================================

IV. PEDAGOGY LAW (NON-NEGOTIABLE TEACHING STYLE)  
The book MUST:  
- Be beginner-first  
- Use simple, clear explanations  
- Always introduce hands-on demos BEFORE theory  
- Use analogies for every complex concept  
- Contain diagrams for every multi-component system  
- Contain short paragraphs only  

Each chapter MUST end with:  
1. Hands-on exercises  
2. A mini robotics task  
3. A 5–10 question quiz  
4. One standalone coding assignment  
============================================================

============================================================
V. TECHNOLOGY STACK RESTRICTION  
The book, tools, examples, and RAG chatbot MUST use ONLY:  

**Documentation:** Docusaurus  
**AI Generation:** Gemini CLI + SpecKit  
**Backend:** FastAPI (Python)  
**Vector DB:** Qdrant Cloud  
**Metadata:** Neon Postgres  
**Chatbot:** OpenAI Agents + ChatKit  
**Hardware:**  
- Jetson Orin Nano/NX  
- RealSense D435i  
- IMU sensors  
- Unitree Go2 or G1 (optional)  
- Digital Twin Workstation (RTX GPU mandatory)

NO other tools, frameworks, SDKs, or languages are allowed unless formally added via constitutional amendment.
============================================================

============================================================
VI. RAG & CHATBOT SAFETY CONSTITUTION  
The built-in RAG assistant MUST obey:

**Allowed Behavior:**  
- Only answer from indexed book content  
- Always show retrieved snippets  
- Always cite chapters/sections  
- Always prioritize safety

**Forbidden Behavior:**  
- Hallucinating content  
- Giving dangerous robotic operation instructions  
- Making hardware claims not present in the book  
- Producing speculative engineering guidance  

**Required Architecture:**  
A multi-agent system including:  
- Retriever Agent  
- Book Reasoner  
- Citation Agent  
- Safety Guard Agent  
============================================================

============================================================
VII. SPEC-DRIVEN DEVELOPMENT WORKFLOW RULES  
All content MUST be created through the following Gemini + SpecKit workflow:

1. `/sp.plan` — Create chapter plan  
2. `/sp.expand` — Expand sections  
3. `/sp.implement` — Generate code, diagrams, examples  
4. `/sp.summarize` — Create chapter summary  

**Violation:** Any content generated outside this pipeline is invalid.  
All prompts MUST follow the template standards in `.specify/templates/`.
============================================================

============================================================
VIII. QUALITY & COMPLETENESS REQUIREMENTS  
Every chapter MUST include:  
- At least one system diagram  
- At least one real code example  
- At least one robotics mini-project  
- A “Test Your Understanding” quiz  
- A “Hands-On Assignment” section  
- Safety notes for robot control  
============================================================

============================================================
IX. DEVELOPER TRANSPARENCY RULES  
The constitution MUST enforce that the book shows:  
- Full FastAPI examples  
- Qdrant schemas  
- Neon Postgres table designs  
- RAG diagrams  
- The multi-agent architecture  
- Embedding, chunking, and retrieval process  

This ensures the book doubles as a reference for developers building similar RAG systems.
============================================================

============================================================
X. GOVERNANCE & AMENDMENT PROCEDURE  
Define:  
- Semantic versioning rules  
- Amendment approval rules  
- Protected non-editable sections  
- Requirements for proposing changes  
============================================================

============================================================
XI. RATIFICATION  
End with:  
- Version number  
- Ratification date  
- Last amended date  
============================================================

Using all details above, generate the final constitution as a strong, formal, hackathon-ready governing document.

## Response snapshot

The project constitution has been created and saved to `.specify/memory/constitution.md` as version 1.0.0.

Here is a suggested commit message:
`docs: establish v1.0.0 of project constitution`

This document now governs all content creation, workflows, and technical standards for the project.

## Outcome

- ✅ Impact: Initial project constitution created. This governs all future work.
- 🧪 Tests: N/A
- 📁 Files: Created .specify/memory/constitution.md
- 🔁 Next prompts: Implement the feature specification based on the updated constitution. I want to build...
- 🧠 Reflection: The user provided a very detailed structure for the constitution, which overrode the existing template. The process was smooth.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment: none
