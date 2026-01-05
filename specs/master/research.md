# Research & Decisions for Physical AI & Humanoid Robotics Book

This document records the research findings for the items marked "NEEDS CLARIFICATION" in the implementation plan.

## 1. Performance Goals

### Docusaurus Frontend

- **Metric**: Page Load Speed (Largest Contentful Paint - LCP)
  - **Goal**: `< 2.5 seconds` on a standard desktop connection.
  - **Rationale**: Provides a good user experience and is recommended by Google's Core Web Vitals.
- **Metric**: Build Time
  - **Goal**: Initial build `< 5 minutes`. Incremental builds `< 10 seconds`.
  - **Rationale**: Fast build times are crucial for developer productivity and rapid content deployment. We will leverage Docusaurus's experimental "faster" mode which uses Rspack.

### FastAPI RAG Backend

- **Metric**: End-to-End Chat Response Latency (p95)
  - **Goal**: `< 3 seconds`
  - **Rationale**: This is a common target for interactive chatbot applications to feel responsive. This breaks down as:
    - **Retrieval Latency (p95)**: `< 300ms` (from vector DB)
    - **LLM Generation Latency (p95)**: `< 2.5s` (first token should be much faster)
    - **API Overhead (p95)**: `< 200ms`
- **Metric**: Throughput
  - **Goal**: Handle `50` concurrent users with the p95 latency goal.
  - **Rationale**: A reasonable starting point for a technical book's audience during a hackathon-level deployment.

## 2. Testing Strategy

### Docusaurus Frontend (Jest)

- **Unit Tests**: For any custom React components created for the Docusaurus site (e.g., custom quiz components, interactive diagrams), we will use `Jest` and `React Testing Library`.
- **Content Linting**: We will use `remark-lint` to enforce markdown style consistency.
- **End-to-End (E2E) Tests**: We will use `Playwright` to test critical user flows, such as navigation, search, and interacting with custom components. This will run against the production build (`npm run build && npm run serve`).
- **Broken Link Checking**: Docusaurus automatically checks for broken links during the build process.

### FastAPI Backend (`pytest`)

- **Unit Tests**: All business logic (e.g., chunking algorithms, agent logic) will be unit-tested with `pytest`. External services (Qdrant, Neon, LLM APIs) will be mocked.
- **Integration Tests**: We will write integration tests that connect to live instances of Qdrant and Neon Postgres running in a test environment (or locally via Docker) to verify data persistence and retrieval logic.
- **Contract Tests**: The OpenAPI schema will be used to generate client code for contract testing, ensuring no breaking changes are introduced to the API. `pytest-https` will be used for this.

**Decision**: The testing strategy is approved. `pytest` for the backend and `Jest`/`Playwright` for the frontend provide comprehensive coverage.
