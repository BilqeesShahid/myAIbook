# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/001-robotics-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Not explicitly requested in spec.md, so no explicit test-writing tasks will be generated. However, testing will be implicitly part of validating each completed user story.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`, `docs/` for Docusaurus.

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Initialize Docusaurus project in the root directory
- [ ] T002 Configure Docusaurus `docusaurus.config.js` for "MyBook" navbar item
- [ ] T003 Initialize Python environment for `backend/` and create `backend/requirements.txt`
- [ ] T004 Create `backend/src/` directory structure (api, services, models)
- [ ] T005 Initialize optional `frontend/` directory structure (components, pages, services)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Setup basic FastAPI application in `backend/src/api/main.py`
- [ ] T007 Configure Docusaurus `sidebar.js` for initial "book title page" and 0-indexed module structure in `docs/`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Interactive Book Navigation (Priority: P1) 🎯 MVP

**Goal**: Implement Docusaurus Navbar, Sidebar, and custom Book Index Page with responsive UI cards.

**Independent Test**: Verify by running the Docusaurus site locally. All navigation elements should function, and the Book Index Page should render correctly with responsive cards displaying module information.

### Implementation for User Story 1

- [ ] T008 [P] [US1] Create custom Docusaurus component for Book Index Page in `src/pages/index.js` (or similar location)
- [ ] T009 [P] [US1] Implement 6 responsive card components (3 rows x 2 columns) within the Book Index Page `src/pages/index.js`
- [ ] T010 [P] [US1] Ensure Module #, Module Name, and chapter bullet lists are displayed in cards 1-5 in `src/pages/index.js`
- [ ] T011 [P] [US1] Implement Assessment & Quiz Strategy content for card 6 in `src/pages/index.js`
- [ ] T012 [P] [US1] Update `docusaurus.config.js` to point "MyBook" to the custom Book Index Page (`src/pages/index.js`)
- [ ] T013 [P] [US1] Implement renaming of "tutorial-info" to "book title page" in `sidebar.js`
- [ ] T014 [P] [US1] Ensure all module and chapter titles in `sidebar.js` match book page titles and are clickable
- [ ] T015 [P] [US1] Verify no empty links or commented items in `sidebar.js`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Comprehensive Content Generation (Priority: P1)

**Goal**: Claude agents generate all required book content (modules, chapters, quizzes, diagrams, exercises, Urdu translations) and deploy the completed book to GitHub Pages.

**Independent Test**: Review all generated book content files in `docs/` for completeness, accuracy, and adherence to constitution rules. Verify successful deployment to GitHub Pages.

### Implementation for User Story 2

- [ ] T016 [US2] Generate placeholder `0-ros2/<chapters>.md` files in `docs/0-ros2/`
- [ ] T017 [US2] Generate placeholder `1-simulation/<chapters>.md` files in `docs/1-simulation/`
- [ ] T018 [US2] Generate placeholder `2-isaac/<chapters>.md` files in `docs/2-isaac/`
- [ ] T019 [US2] Generate placeholder `3-vla/<chapters>.md` files in `docs/3-vla/`
- [ ] T020 [US2] Generate placeholder `4-capstone/<chapters>.md` files in `docs/4-capstone/`
- [ ] T021 [US2] Update `sidebar.js` with all generated modules and chapters, ensuring 0-indexed modules
- [ ] T022 [US2] Implement Claude agent logic for generating accurate ROS2 Fundamentals content
- [ ] T023 [US2] Implement Claude agent logic for generating Robot Simulation content (Gazebo & Unity)
- [ ] T024 [US2] Implement Claude agent logic for generating AI-Robot Brain content (NVIDIA Isaac)
- [ ] T025 [US2] Implement Claude agent logic for generating Vision-Language-Action (VLA) content
- [ ] T026 [US2] Implement Claude agent logic for generating Capstone Project content
- [ ] T027 [US2] Implement Claude agent logic for generating quizzes (MCQs, Code Completion, Simulation Tasks) for each module
- [ ] T028 [US2] Implement Claude agent logic for generating diagrams and exercises
- [ ] T029 [US2] Implement Claude agent logic for generating Urdu translations of content
- [ ] T030 [US2] Configure Docusaurus for GitHub Pages deployment (update `docusaurus.config.js` and `.github/workflows/deploy.yml` if applicable)
- [ ] T031 [US2] Deploy the completed Docusaurus book to GitHub Pages

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - RAG Chatbot Integration (Priority: P2)

**Goal**: Integrate a Qdrant-powered RAG chatbot with the deployed book content, providing accurate and context-aware answers, including multi-turn conversations and Urdu output.

**Independent Test**: Interact with the deployed chatbot, send queries based on book content, and verify accuracy, multi-turn context, and Urdu translation.

### Implementation for User Story 3

- [ ] T032 [US3] Configure Qdrant Cloud Free Tier for vector database in `backend/src/services/qdrant_service.py`
- [ ] T033 [US3] Implement logic to extract content from deployed book URLs in `backend/src/services/content_extractor.py`
- [ ] T034 [US3] Implement logic to create embeddings and store in Qdrant in `backend/src/services/qdrant_service.py`
- [ ] T035 [US3] Implement OpenAI Agents SDK integration in `backend/src/services/rag_agent.py`
- [ ] T036 [US3] Create `POST /ask` endpoint in `backend/src/api/routes.py`
- [ ] T037 [US3] Create `POST /ask/selected-text` endpoint in `backend/src/api/routes.py`
- [ ] T038 [US3] Implement retrieval logic for RAG chatbot to answer only from deployed content in `backend/src/services/rag_agent.py`
- [ ] T039 [US3] Implement logic to prioritize selected text for retrieval in `backend/src/services/rag_agent.py`
- [ ] T040 [US3] Implement multi-turn conversation context management in `backend/src/services/rag_agent.py`
- [ ] T041 [US3] Implement Urdu output translation for chatbot responses in `backend/src/services/translation_service.py`

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T042 Code cleanup and refactoring across `docs/`, `backend/`, and `frontend/` (if applicable)
- [ ] T043 Performance optimization for FastAPI endpoints in `backend/src/api/`
- [ ] T044 Security hardening for FastAPI backend in `backend/src/api/`
- [ ] T045 Update `README.md` with deployment instructions and feature overview

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Depends on User Story 2 completion (deployed book content)

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks (T001-T005) marked [P] can run in parallel (implicitly, as they are separate setup steps).
- Once Foundational phase (T006-T007) completes, User Story 1 (T008-T015) tasks can largely proceed in parallel, as many involve separate UI components or config updates.
- User Story 2 tasks (T016-T031) involve agent logic and content generation, which can be parallelized if separate agents handle different module generations.
- User Story 3 tasks (T032-T041) for backend development can also be parallelized where dependencies allow (e.g., Qdrant setup, content extraction, and agent integration could be somewhat independent initially).

---

## Parallel Example: User Story 1

```bash
# Launch UI components for User Story 1 together:
Task: "Implement 6 responsive card components (3 rows x 2 columns) within the Book Index Page src/pages/index.js"
Task: "Ensure Module #, Module Name, and chapter bullet lists are displayed in cards 1-5 in src/pages/index.js"
Task: "Implement Assessment & Quiz Strategy content for card 6 in src/pages/index.js"
```

---

## Implementation Strategy

### MVP First (User Story 1 + 2 only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Interactive Book Navigation)
4. Complete Phase 4: User Story 2 (Comprehensive Content Generation)
5. **STOP and VALIDATE**: Test User Stories 1 & 2 independently, verify book deployment.
6. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Docusaurus UI)
   - Developer B: User Story 2 (Content Generation & Deployment)
   - Developer C: User Story 3 (RAG Chatbot Backend)
3. Stories complete and integrate independently (Note: US3 depends on US2 for deployed content)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
