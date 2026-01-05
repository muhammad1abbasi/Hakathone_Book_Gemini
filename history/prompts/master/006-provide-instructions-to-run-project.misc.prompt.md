---
id: 006
title: Provide Instructions to Run Project
stage: misc
date: 2025-12-29
surface: agent
model: gemini-pro
feature: master
branch: master
user: user
command: User asked how to run the project.
labels: ["instructions", "quickstart", "docusaurus"]
links:
  spec: specs/master/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/master/quickstart.md
tests:
  - none
---

## Prompt

now how to run this project how to see this book on docousorse but be care full dont erase or change any progress that we made

## Response snapshot

I have provided the full set of instructions to the user. I have updated the `quickstart.md` file and output its content.

## Outcome

- ✅ Impact: Provided clear, updated instructions for running the Docusaurus frontend.
- 🧪 Tests: N/A
- 📁 Files: Modified `specs/master/quickstart.md`.
- 🔁 Next prompts: User will likely proceed with setting up and running the project.
- 🧠 Reflection: It was crucial to correct the initial `quickstart.md` instructions given the current state of the Docusaurus project and generated content.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- next_experiment: none
