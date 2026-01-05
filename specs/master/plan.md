# Implementation Plan: Physical AI & Humanoid Robotics Book

**Branch**: `master` | **Date**: 2025-12-29 | **Spec**: [specs/master/spec.md](specs/master/spec.md)
**Input**: Feature specification from `specs/master/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The project is to build a Docusaurus-based technical book, "Physical AI & Humanoid Robotics — From Digital Brain to Embodied Intelligence," and an accompanying AI-powered RAG chatbot backend. The book will teach students how to control humanoid robots in simulation and the real world, using a strictly defined, modern technology stack. The entire project will be developed following a Spec-Driven Development workflow.

## Technical Context

**Language/Version**: Python 3.11+, Node.js/TypeScript (for Docusaurus)
**Primary Dependencies**: Docusaurus, FastAPI, Qdrant Cloud, Neon Postgres, OpenAI Whisper, ROS 2, Gazebo, NVIDIA Isaac
**Storage**: Qdrant Cloud (Vector DB), Neon Postgres (Serverless for structured metadata)
**Testing**: `pytest` for backend, `Jest` for frontend (Docusaurus)
**Target Platform**: Web (Docusaurus site), Linux Server (FastAPI backend), Robotics simulation (Windows/Linux with RTX GPU), Embedded (NVIDIA Jetson Orin Nano/NX)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: RAG chatbot response time < 3 seconds p95. NEEDS CLARIFICATION on other goals.
**Constraints**: All development must adhere strictly to the technology stack defined in the Constitution. Hardware requirements for the digital twin workstation (RTX GPU) and robotics (Jetson Orin) must be clearly documented.
**Scale/Scope**: The book will consist of 4 modules, 16 chapters, and 64 lessons, plus a full RAG backend.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **[PASS] Section II: Scope Doctrine**: The plan directly implements the module and chapter structure defined in the constitution.
- **[PASS] Section III: Target Audience Mandate**: The pedagogical approach is designed for beginners and intermediate learners.
- **[PASS] Section IV: Pedagogy Law**: The plan must ensure that all content generation tasks include the required hands-on demos, analogies, diagrams, exercises, quizzes, and assignments. This will be enforced in the task generation phase.
- **[PASS] Section V: Technology Stack Restriction**: The plan exclusively uses the technologies mandated in the constitution.
- **[PASS] Section VI: RAG & Chatbot Safety Constitution**: The backend plan includes the required endpoints. The implementation must include the specified multi-agent architecture (Retriever, Reasoner, Citation, Safety).
- **[PASS] Section VII: Spec-Driven Development Workflow**: This plan is being generated via the `/sp.plan` command as required. The user has explicitly forbidden the use of `/sp.expand` and `/sp.implement` in this planning phase.
- **[PASS] Section VIII: Quality & Completeness Requirements**: The plan acknowledges that every chapter must include diagrams, code, projects, quizzes, and safety notes.
- **[PASS] Section IX: Developer Transparency Rules**: The plan for the backend includes providing full-source examples and schema designs as required.

## Project Structure

### Documentation (this feature)

```text
specs/master/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── openapi.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application (frontend + backend) structure

# Frontend: Docusaurus Book Content
docs/
  intro.md
  module-01-ros2/
    chapter-01-foundations/
      lesson-01-what-is-physical-ai.md
      lesson-02-embodied-intelligence.md
      lesson-03-human-vs-robot-agency.md
      lesson-04-sensor-overview.md
    chapter-02-ros2-core/
      lesson-01-nodes.md
      lesson-02-topics.md
      lesson-03-services.md
      lesson-04-actions.md
    chapter-03-rclpy-bridge/
      lesson-01-python-to-ros.md
      lesson-02-robot-controllers.md
      lesson-03-message-types.md
      lesson-04-debugging-tools.md
    chapter-04-urdf-design/
      lesson-01-urdf-basics.md
      lesson-02-humanoid-joints.md
      lesson-03-sensors-in-urdf.md
      lesson-04-testing-models.md
  module-02-simulation/
    # ... (content as per spec)
  module-03-nvidia-isaac/
    # ... (content as per spec)
  module-04-vla/
    # ... (content as per spec)

# Frontend: Docusaurus Application
docusaurus/
  ├── docusaurus.config.ts
  ├── package.json
  └── src/
      ├── css/
      ├── pages/
      └── theme/

# Backend: FastAPI RAG Service
backend/
├── src/
│   ├── api/
│   │   ├── endpoints/
│   │   │   ├── embed.py
│   │   │   ├── search.py
│   │   │   └── chat.py
│   │   └── main.py
│   ├── core/
│   │   ├── config.py
│   │   └── security.py
│   ├── models/
│   │   ├── chat.py
│   │   └── user.py
│   ├── services/
│   │   ├── chunking.py
│   │   ├── embedding.py
│   │   └── vector_db.py
│   └── agents/
│       ├── retriever_agent.py
│       ├── reasoner_agent.py
│       ├── citation_agent.py
│       └── safety_guard_agent.py
└── tests/
    ├── integration/
    └── unit/

```

**Structure Decision**: A hybrid structure is adopted. The book content resides in the top-level `docs/` directory as required by Docusaurus. The Docusaurus application itself will be set up in a `docusaurus/` directory. The RAG backend is a standard FastAPI application housed in the `backend/` directory. This cleanly separates the content, the frontend presentation layer, and the backend service logic.

## Complexity Tracking

No violations to the constitution were identified. This section is not required.