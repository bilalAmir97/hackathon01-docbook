# Tasks: Embedding Pipeline Setup

**Input**: Design documents from /specs/004-Spec01-embedding-pipeline/
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not explicitly requested - implementation tasks only.

**Organization**: Tasks grouped by user story for independent implementation and testing.

## Path Conventions

- **Single project**: backend/ at repository root (single-file main.py)
- All functions in backend/main.py per user requirement

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Initialize Python project with uv and configure dependencies

- [x] T001 Create backend/ directory at repository root
- [x] T002 Initialize Python project with uv in backend/ (uv init)
- [x] T003 [P] Create backend/.python-version with 3.11
- [x] T004 [P] Create backend/.env.example with required environment variables
- [x] T005 Add dependencies to backend/pyproject.toml: cohere, qdrant-client, httpx, beautifulsoup4, lxml, python-dotenv

**Checkpoint**: Project structure ready, dependencies installable with uv sync

---

## Phase 2: Foundational (Core Functions)

**Purpose**: Implement helper functions that ALL user stories depend on

- [x] T006 Create backend/main.py with imports and docstring header
- [x] T007 [P] Implement compute_content_hash() in backend/main.py
- [x] T008 [P] Implement compute_point_id() in backend/main.py
- [x] T009 Implement chunk_text() in backend/main.py
- [x] T010 Implement get_all_urls() in backend/main.py
- [x] T011 Implement extract_text_from_url() in backend/main.py
- [x] T012 Implement create_collection() in backend/main.py
- [x] T013 Implement embed() in backend/main.py
- [x] T014 Implement save_chunk_to_qdrant() in backend/main.py

**Checkpoint**: All helper functions ready

---

## Phase 3: User Story 1 - Initial Pipeline Execution (P1) MVP

**Goal**: Run pipeline so all content becomes searchable

- [x] T015 [US1] Implement main() skeleton in backend/main.py
- [x] T016 [US1] Add URL discovery loop in main()
- [x] T017 [US1] Add text extraction in main() loop
- [x] T018 [US1] Add chunking in main() loop
- [x] T019 [US1] Add embedding generation in main() loop
- [x] T020 [US1] Add vector storage in main() loop
- [x] T021 [US1] Add progress logging in main()
- [x] T022 [US1] Add error handling in main()
- [x] T023 [US1] Add if __name__ == __main__ block

**Checkpoint**: Pipeline runs end-to-end

---

## Phase 4: User Story 2 - Idempotency (P1)

**Goal**: Re-run pipeline without duplicates

- [x] T024 [US2] Implement check_content_changed() in backend/main.py
- [x] T025 [US2] Add content hash computation in main() loop
- [x] T026 [US2] Add hash comparison logic in main() loop
- [x] T027 [US2] Add vectors_unchanged counter
- [x] T028 [US2] Update main() to use deterministic point IDs

**Checkpoint**: Re-runs are idempotent

---

## Phase 5: User Story 3 - Metadata (P2)

**Goal**: Each vector contains source metadata

- [x] T029 [US3] Extract section headings in extract_text_from_url()
- [x] T030 [US3] Add section_heading to chunk metadata
- [x] T031 [US3] Ensure payload includes all required fields
- [x] T032 [US3] Add indexed_at timestamp to payload

**Checkpoint**: All vectors have complete metadata

---

## Phase 6: User Story 4 - Reporting (P3)

**Goal**: Generate summary report after execution

- [x] T033 [US4] Create RunReport structure in backend/main.py
- [x] T034 [US4] Add statistics tracking throughout main()
- [x] T035 [US4] Add failure tracking in main()
- [x] T036 [US4] Add timing tracking
- [x] T037 [US4] Generate run_id at pipeline start
- [x] T038 [US4] Build run report dict at end of main()
- [x] T039 [US4] Print run report as JSON to stdout

**Checkpoint**: Pipeline outputs complete JSON report

---

## Phase 7: Polish

**Purpose**: Final improvements

- [x] T040 Add environment variable validation
- [x] T041 Add Qdrant client timeout (30s)
- [x] T042 Add rate limiting (2 req/s)
- [x] T043 Add exponential backoff for Cohere
- [x] T044 Add error threshold check (5%)
- [x] T045 Run quickstart.md validation
- [x] T046 Final code cleanup

---

## Summary

| Phase | Tasks | Story | Description |
|-------|-------|-------|-------------|
| 1 | T001-T005 | Setup | Project initialization |
| 2 | T006-T014 | Foundational | Core functions |
| 3 | T015-T023 | US1 (P1) | Initial Pipeline |
| 4 | T024-T028 | US2 (P1) | Idempotency |
| 5 | T029-T032 | US3 (P2) | Metadata |
| 6 | T033-T039 | US4 (P3) | Reporting |
| 7 | T040-T046 | Polish | Cross-cutting |

**Total Tasks**: 46
**MVP Tasks**: T001-T023 (23 tasks)
