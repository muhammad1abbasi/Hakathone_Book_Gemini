# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `specs/master/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/, research.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: User story (e.g., US1, US2)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization for both Docusaurus frontend and FastAPI backend.

- [ ] T001 [P] Create the main project directory structure: `docusaurus/`, `backend/`, `docs/`.
- [ ] T002 [P] Initialize a `git` repository in the root directory.
- [ ] T003 [P] Create a `.gitignore` file with standard ignores for Python and Node.js.

---

## Phase 2: User Story 1 - Docusaurus Site and Content Structure (Priority: P1) 🎯 MVP

**Goal**: A fully structured Docusaurus site with all markdown files for the book scaffolded and ready for content. The site should be viewable locally.

**Independent Test**: Run `npm run start` in the `docusaurus/` directory and verify that the site loads with the complete sidebar structure, showing all modules, chapters, and lessons, even if the pages are empty.

### Implementation for User Story 1

- [ ] T004 [US1] Initialize the Docusaurus project in the `docusaurus/` directory using `npm create docusaurus@latest docusaurus -- --typescript`.
- [ ] T005 [US1] Configure `docusaurus/docusaurus.config.ts` with the book title "Physical AI & Humanoid Robotics", theme, and plugins.
- [ ] T006 [US1] Add syntax highlighting for Python and C++ in `docusaurus/docusaurus.config.ts`.
- [ ] T007 [US1] Configure Algolia DocSearch in `docusaurus/docusaurus.config.ts`.
- [ ] T008 [US1] Configure the sidebar structure in `docusaurus/sidebars.ts` to manually reflect the 4 modules, 16 chapters, and 64 lessons.
- [ ] T009 [US1] Configure the versioning system in `docusaurus/docusaurus.config.ts` to manage book editions.
- [ ] T010 [P] [US1] Create the directory structure inside `docs/` for `module-01-ros2` as specified in the plan.
- [ ] T011 [P] [US1] Create the directory structure inside `docs/` for `module-02-simulation` as specified in the plan.
- [ ] T012 [P] [US1] Create the directory structure inside `docs/` for `module-03-nvidia-isaac` as specified in the plan.
- [ ] T013 [P] [US1] Create the directory structure inside `docs/` for `module-04-vla` as specified in the plan.
- [ ] T014 [P] [US1] Create placeholder markdown files for all 16 lessons in `module-01-ros2` with basic frontmatter (id, title).
- [ ] T015 [P] [US1] Create placeholder markdown files for all 16 lessons in `module-02-simulation` with basic frontmatter.
- [ ] T016 [P] [US1] Create placeholder markdown files for all 16 lessons in `module-03-nvidia-isaac` with basic frontmatter.
- [ ] T017 [P] [US1] Create placeholder markdown files for all 16 lessons in `module-04-vla` with basic frontmatter.
- [ ] T018 [US1] Create `docs/intro.md`.

---

## Phase 3: User Story 2 - RAG Backend Core Implementation (Priority: P2)

**Goal**: A functional FastAPI backend that can embed text, perform vector searches, and respond to chat messages using a simple RAG loop.

**Independent Test**: Send a POST request to `/chat` with a query. Verify that the API returns a 200 response containing a generated answer and a list of sources retrieved from the vector database.

### Implementation for User Story 2

- [ ] T019 [US2] Initialize the FastAPI project in the `backend/` directory with a `pyproject.toml` or `requirements.txt`.
- [ ] T020 [US2] Create the backend directory structure (`src/api`, `src/core`, `src/models`, `src/services`, `src/agents`, `tests/`) as defined in `plan.md`.
- [ ] T021 [US2] Implement the Neon Postgres schema from `data-model.md` using SQL scripts or an alembic migration in `backend/src/core/db.py`.
- [ ] T022 [US2] Configure the Qdrant collection settings (name, vector size, distance metric) in `backend/src/core/config.py`.
- [ ] T023 [US2] Implement the text chunking logic in `backend/src/services/chunking.py`.
- [ ] T024 [US2] Implement the embedding pipeline using an external model (e.g., from OpenAI or a Hugging Face sentence-transformer) in `backend/src/services/embedding.py`.
- [ ] T025 [US2] Implement the `/embed` endpoint in `backend/src/api/endpoints/embed.py` that chunks and embeds text, then stores it in Qdrant.
- [ ] T026 [US2] Implement the `/search` endpoint in `backend/src/api/endpoints/search.py` that performs a vector search in Qdrant.
- [ ] T027 [US2] Implement the `/chat` endpoint in `backend/src/api/endpoints/chat.py` that orchestrates the search and generation loop.
- [ ] T028 [US2] Implement the RAG agent logic (Retriever, Reasoner, Citation, Safety) within the `backend/src/agents/` directory.

---

## Phase 4: User Story 3 - Frontend Chatbot UI Integration (Priority: P3)

**Goal**: A chat interface embedded within the Docusaurus site that can communicate with the FastAPI backend.

**Independent Test**: Open the Docusaurus site, open the chat widget, and ask a question. Verify that a response from the AI is displayed in the UI.

### Implementation for User Story 3

- [ ] T029 [US3] Create a new React component for the chat widget in `docusaurus/src/theme/ChatWidget.tsx`.
- [ ] T030 [US3] "Swaizzle" the Docusaurus theme's root layout to include the `ChatWidget` component in `docusaurus/src/theme/Root.tsx`.
- [ ] T031 [US3] Implement the UI for displaying messages, a text input, and a send button in `docusaurus/src/theme/ChatWidget.tsx`.
- [ ] T032 [US3] Implement API client logic within `ChatWidget.tsx` to send requests to the FastAPI backend's `/chat` endpoint.
- [ ] T033 [US3] Add loading and error state UI to the chat widget in `docusaurus/src/theme/ChatWidget.tsx`.
- [ ] T034 [US3] Implement a feature where a user can select text on a page and click a button to ask the chatbot a question about it.

---

## Phase 5: User Story 4 - Hackathon Delivery (Priority: P4)

**Goal**: Prepare the project for final demonstration and judging.

**Independent Test**: The full demo script can be executed without errors. The project is deployed to a public URL.

### Implementation for User Story 4

- [ ] T035 [US4] Create a detailed demo script in `demo-script.md`.
- [ ] T036 [US4] Pre-populate the RAG system with content from at least one full module of the book.
- [ ] T037 [P] [US4] Deploy the Docusaurus site to a static hosting provider (e.g., Vercel, Netlify).
- [ ] T038 [P] [US4] Deploy the FastAPI backend to a cloud service (e.g., Fly.io, Railway, or a cloud VM).
- [ ] T039 [US4] Create a fallback plan with a local video recording of the demo in case of live demo failure.
- [ ] T040 [US4] Prepare a presentation slide deck summarizing the project's architecture, features, and accomplishments.

## Dependencies & Execution Order

- **User Story 1**: Can start immediately.
- **User Story 2**: Can start in parallel with User Story 1.
- **User Story 3**: Depends on the completion of US1 (for the site to embed into) and US2 (for the API to call).
- **User Story 4**: Depends on the completion of US1, US2, and US3.
