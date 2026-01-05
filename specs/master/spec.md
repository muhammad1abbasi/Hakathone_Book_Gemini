You are generating a production-grade, hackathon-winning development plan.

This plan MUST be extremely explicit, operational, and engineer-level.
No vague wording is allowed. No summaries. No abstractions.

Book Title:
"Physical AI & Humanoid Robotics — From Digital Brain to Embodied Intelligence"

Primary Goal:
Build a Docusaurus-based technical book AND an AI-powered RAG chatbot backend that teaches students how to control humanoid robots in simulation and real-world workflows.

TECH STACK (MANDATORY):
- Docusaurus
- FastAPI (Python 3.11+)
- Qdrant Cloud
- Neon Postgres (Serverless)
- OpenAI Whisper
- ROS 2 + Gazebo + NVIDIA Isaac (content only)

────────────────────────────────────────────
SECTION 1: BOOK STRUCTURE (EXPLICIT)

The book MUST follow this structure:

TOTAL MODULES: 4  
CHAPTERS PER MODULE: 4  
LESSONS PER CHAPTER: 4  

Use this EXACT structure and naming format:

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
    chapter-01-gazebo-basics/
      lesson-01-installation.md
      lesson-02-world-building.md
      lesson-03-gravity-and-collisions.md
      lesson-04-plugin-system.md
    chapter-02-unity-visualization/
      lesson-01-unity-setup.md
      lesson-02-human-robot-interaction.md
      lesson-03-animation-sync.md
      lesson-04-render-pipelines.md
    chapter-03-sensor-simulation/
      lesson-01-lidar-sim.md
      lesson-02-depth-cameras.md
      lesson-03-imu-sim.md
      lesson-04-noise-models.md
    chapter-04-digital-twin/
      lesson-01-environment-replication.md
      lesson-02-domain-randomization.md
      lesson-03-performance-optimization.md
      lesson-04-validation-testing.md

  module-03-nvidia-isaac/
    chapter-01-isaac-sim/
      lesson-01-installation.md
      lesson-02-omniverse-assets.md
      lesson-03-synthetic-data.md
      lesson-04-domain-lighting.md
    chapter-02-isaac-ros/
      lesson-01-vslam.md
      lesson-02-hardware-acceleration.md
      lesson-03-camera-pipelines.md
      lesson-04-gpu-offloading.md
    chapter-03-nav2/
      lesson-01-path-planning.md
      lesson-02-bipedal-walking.md
      lesson-03-obstacle-avoidance.md
      lesson-04-recovery-behaviors.md
    chapter-04-sim-to-real/
      lesson-01-transfer-learning.md
      lesson-02-noise-injection.md
      lesson-03-hardware-constraints.md
      lesson-04-real-world-validation.md

  module-04-vla/
    chapter-01-voice/
      lesson-01-whisper-integration.md
      lesson-02-audio-preprocessing.md
      lesson-03-wake-words.md
      lesson-04-latency-optimization.md
    chapter-02-language/
      lesson-01-intent-detection.md
      lesson-02-task-parsing.md
      lesson-03-sequence-generation.md
      lesson-04-error-correction.md
    chapter-03-action/
      lesson-01-ros2-task-execution.md
      lesson-02-motion-primitives.md
      lesson-03-grasp-planning.md
      lesson-04-failure-handling.md
    chapter-04-capstone/
      lesson-01-full-pipeline.md
      lesson-02-testing-sim.md
      lesson-03-real-hardware.md
      lesson-04-demo-preparation.md

────────────────────────────────────────────
SECTION 2: DOCUSAURUS PLAN (MANDATORY DETAIL)

The plan must include:
- npm create docusaurus@latest
- TypeScript configuration
- docusaurus.config.ts configuration
- Sidebar auto-generation config
- Versioning setup

Include real command examples.

────────────────────────────────────────────
SECTION 3: FASTAPI + RAG BACKEND PLAN

Plan MUST include:

FastAPI Endpoints:
- POST /embed
- POST /search
- POST /chat

Qdrant:
- Collection name
- Vector size
- Distance metric
- Payload schema

Neon Postgres:
- Full SQL schema for:
  - users
  - chats
  - message_logs

Chunking Strategy:
- Chunk size in tokens
- Overlap size
- Metadata tracking

────────────────────────────────────────────
SECTION 4: HACKATHON EXECUTION PLAN

Break down:
Day 1-2: Environment setup
Day 3-5: Content scaffolding
Day 6-8: Backend development
Day 9-10: RAG integration
Day 11-12: UI integration
Day 13-14: Demo + polishing

────────────────────────────────────────────

RULES:
- Do NOT use sp.expand
- Do NOT use sp.implement
- No vagueness
- No generic filler content
- Must be judge-ready

Now generate the plan.
