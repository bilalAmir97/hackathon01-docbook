# Tasks: RAG Agent API (Spec-3)

**Input**: Design documents from `/specs/006-rag-agent-api/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/api-contracts.md, research.md, quickstart.md

**Tests**: Test tasks are included for critical paths. Each task has explicit acceptance criteria.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1, US2, US3, US4)
- Exact file paths included in descriptions

## Path Conventions

- **Backend**: `backend/` at repository root (matches Spec-2 structure)
- **Tests**: `backend/tests/`
- **Specs**: `specs/006-rag-agent-api/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and base configuration

- [X] T001 Update pyproject.toml with Spec-3 dependencies in `backend/pyproject.toml`
  - Add: `openai-agents`, `fastapi>=0.109`, `uvicorn[standard]>=0.27`, `sse-starlette>=1.8`, `asyncpg>=0.29`, `pydantic>=2.0`, `structlog>=24.1`, `httpx>=0.27`
  - Add test deps: `pytest>=8.0`, `pytest-asyncio>=0.23`
  - **Accept**: `uv pip install -e ".[test]"` succeeds

- [X] T002 [P] Create environment configuration in `backend/.env.example`
  - Include: GEMINI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, GEMINI_MODEL, LOG_LEVEL
  - **Accept**: All required env vars documented with examples

- [X] T003 [P] Create Pydantic request/response models in `backend/api_models.py`
  - ChatRequest with validation (query, session_id, mode, selected_text, top_k, score_threshold, filters)
  - SourceCitation, ResponseMetadata, ChatResponse
  - ErrorResponse, ServiceStatus, HealthResponse
  - StreamChunk, StreamSources, StreamDone, StreamError
  - **Accept**: All models from data-model.md implemented with Field validators

- [X] T004 Create FastAPI application scaffold in `backend/api.py`
  - Initialize FastAPI app with title, description, version
  - Add CORS middleware configuration
  - Import placeholder routes (to be implemented)
  - **Accept**: `uvicorn api:app --reload` starts without errors

**Checkpoint**: Setup complete - base project structure ready

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Spec-2 integration verification and core agent infrastructure

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Verify Spec-2 retrieval integration in `backend/` (no new file)
  - Import from retrieval.py: `retrieve`, `retrieve_with_retry`, `embed_query`
  - Import from models.py: `RetrievalRequest`, `SearchResult`, `ChunkResult`
  - Import from errors.py: `EmbeddingError`, `QdrantConnectionError`, `QdrantTimeoutError`
  - Test: Create RetrievalRequest, call retrieve(), verify ChunkResult[] returned
  - **Accept**: `python -c "from retrieval import retrieve, retrieve_with_retry; print('OK')"` succeeds

- [X] T006 [P] Create base agent module scaffold in `backend/agent.py`
  - Import OpenAI Agents SDK: `Agent`, `function_tool`, `Runner`, `set_tracing_disabled`
  - Import LitellmModel from `agents.extensions.models.litellm_model`
  - Call `set_tracing_disabled(disabled=True)`
  - Define GENERAL_SYSTEM_PROMPT constant
  - Define SELECTED_TEXT_SYSTEM_PROMPT constant with `{selected_text}` placeholder
  - **Accept**: `python -c "from agent import GENERAL_SYSTEM_PROMPT; print('OK')"` succeeds

- [X] T007 Create model provider configuration in `backend/agent.py`
  - Implement `get_model()` function returning LitellmModel
  - Use `gemini/gemini-2.0-flash` as default model
  - Read GEMINI_API_KEY from environment
  - **Accept**: `get_model()` returns LitellmModel instance

**Checkpoint**: Foundation ready - agent module initialized, Spec-2 integration verified

---

## Phase 3: User Story 1 - General Question Answering with Citations (Priority: P1) 🎯 MVP

**Goal**: Agent answers questions using Qdrant retrieval with proper citations

**Independent Test**: POST /chat with `mode: "general"` returns answer + sources[]

### Implementation for User Story 1

- [X] T008 [US1] Implement @function_tool search_documentation in `backend/agent.py`
  - Decorator: `@function_tool`
  - Parameters: `query: Annotated[str, "..."]`, `top_k: Annotated[int, "..."] = 5`
  - Create RetrievalRequest, call retrieve()
  - Return list[dict] with: source_url, page_title, section_heading, chunk_text[:2000], relevance_score
  - **Accept**: Tool callable, returns chunk data from Qdrant

- [X] T009 [US1] Implement run_agent() for general mode in `backend/agent.py`
  - Async function: `run_agent(query, mode, selected_text=None, top_k=5, session_id=None)`
  - For mode="general": Create Agent with GENERAL_SYSTEM_PROMPT, tools=[search_documentation]
  - If session_id provided: retrieve conversation history via `get_conversation_history(session_id)`
  - Build message list: previous exchanges + current query (respect MAX_CONVERSATION_HISTORY=10)
  - Call `await Runner.run(agent, input=messages)` with conversation context
  - Extract sources from result.run_items (tool outputs)
  - Return tuple: (result.final_output, sources)
  - **Accept**: `await run_agent("What is ROS 2?", "general")` returns (answer, sources[])
  - **Accept**: Multi-turn: second query in same session includes prior context

- [X] T010 [US1] Implement extract_sources_from_result() helper in `backend/agent.py`
  - Iterate result.run_items
  - Find items with output attribute that is list
  - Collect and return all sources
  - **Accept**: Function extracts tool outputs from RunResult

- [X] T011 [US1] Implement POST /chat endpoint in `backend/api.py`
  - Route: `@app.post("/chat", response_model=ChatResponse)`
  - Accept ChatRequest body
  - Generate trace_id (uuid4[:8])
  - Measure elapsed time
  - Call run_agent() with request params
  - Convert sources to SourceCitation list
  - Return ChatResponse with answer, sources, mode, metadata
  - Handle exceptions: return HTTPException with ErrorResponse
  - **Accept**: `curl -X POST localhost:8000/chat -d '{"query": "What is ROS 2?"}' ` returns JSON with answer and sources

- [X] T012 [US1] Add citation formatting helper in `backend/api.py`
  - Function: `format_citations(sources: list[dict]) -> list[SourceCitation]`
  - Truncate chunk_text to 500 chars
  - Validate relevance_score bounds (0.0-1.0)
  - **Accept**: Raw sources converted to SourceCitation list

**Checkpoint**: User Story 1 complete - General Q&A with citations working

---

## Phase 4: User Story 2 - Selected-Text-Only Mode (Priority: P1)

**Goal**: Agent answers from user-provided text only, no Qdrant retrieval

**Independent Test**: POST /chat with `mode: "selected_text"` + `selected_text` returns answer grounded only in selection

### Implementation for User Story 2

- [X] T013 [US2] Implement selected-text agent configuration in `backend/agent.py`
  - In run_agent(): if mode="selected_text", create Agent with:
    - instructions=SELECTED_TEXT_SYSTEM_PROMPT.format(selected_text=selected_text)
    - tools=[] (empty - no retrieval)
  - Create synthetic source: {"source_url": "selected_text", "page_title": "User Selection", ...}
  - **Accept**: Agent created without tools for selected_text mode

- [X] T014 [US2] Add mode routing in run_agent() in `backend/agent.py`
  - Branch on mode parameter at function start
  - "selected_text" → no tools, SELECTED_TEXT_SYSTEM_PROMPT
  - "general" → search_documentation tool, GENERAL_SYSTEM_PROMPT
  - **Accept**: Both modes create correct agent configuration

- [X] T015 [US2] Add selected_text validation in `backend/api.py`
  - In /chat endpoint: validate mode="selected_text" requires selected_text
  - Return 400 with validation_error if missing
  - **Accept**: Missing selected_text returns proper error response

**Checkpoint**: User Story 2 complete - Selected-text mode working independently

---

## Phase 5: User Story 3 - Chat API with Response Streaming (Priority: P2)

**Goal**: Stream responses via Server-Sent Events for real-time UI updates

**Independent Test**: POST /chat/stream returns SSE events with chunks, sources, done

### Implementation for User Story 3

- [X] T016 [P] [US3] Create SSE event formatting utilities in `backend/api.py`
  - Function: `format_sse_event(data: dict) -> str`
  - Format: `data: {json}\n\n`
  - **Accept**: Events properly formatted for SSE protocol

- [X] T017 [US3] Implement run_agent_streamed() in `backend/agent.py`
  - Async generator function
  - Use `Runner.run_streamed(agent, input=query)`
  - Iterate `result.stream_events()`
  - Yield {"type": "chunk", "content": delta} for ResponseTextDeltaEvent
  - Capture sources from tool_call_output_item
  - Yield {"type": "sources", "sources": [...]} at end
  - Yield {"type": "done", "metadata": {...}} final event
  - **Accept**: Generator yields proper event sequence

- [X] T018 [US3] Implement POST /chat/stream endpoint in `backend/api.py`
  - Route: `@app.post("/chat/stream")`
  - Accept ChatRequest body
  - Return StreamingResponse with media_type="text/event-stream"
  - Call run_agent_streamed() and yield formatted SSE events
  - Handle errors: yield error event, then close stream
  - **Accept**: curl with `-N` flag receives progressive SSE events

**Checkpoint**: User Story 3 complete - Streaming responses working

---

## Phase 6: User Story 4 - Configurable Retrieval Parameters (Priority: P3)

**Goal**: Allow per-request tuning of retrieval behavior

**Independent Test**: POST /chat with custom top_k/score_threshold/filters affects retrieval

### Implementation for User Story 4

- [X] T019 [US4] Pass retrieval parameters through agent in `backend/agent.py`
  - Modify search_documentation to accept top_k from agent context
  - Pass score_threshold to RetrievalRequest if provided
  - Pass filters to RetrievalRequest if provided
  - **Accept**: Different top_k values return different chunk counts

- [X] T020 [US4] Add filter validation in `backend/api.py`
  - Validate filters dict keys: source_url, page_title, section_heading
  - Validate filter values: string or list[str]
  - Return 400 for invalid filter structure
  - **Accept**: Invalid filters rejected with validation_error

**Checkpoint**: User Story 4 complete - Configurable parameters working

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Health checks, error handling, logging, database, end-to-end testing

- [X] T021 [P] Implement GET /health endpoint in `backend/api.py`
  - Check Qdrant connectivity (get_collections)
  - Check Cohere API (test embed)
  - Measure latencies for each service
  - Return HealthResponse with service statuses
  - Overall status: healthy/degraded/unhealthy based on service states
  - **Accept**: /health returns JSON with service status and latencies

- [X] T022 [P] Implement citation validation in `backend/agent.py`
  - Function: `validate_citations(response_sources, retrieved_chunks)`
  - Build set of valid source_urls from retrieved chunks
  - Filter response_sources to only valid URLs + "selected_text"
  - Log warning for stripped citations
  - **Accept**: Invalid citations logged and removed

- [X] T023 [P] Implement error taxonomy handlers in `backend/api.py`
  - Exception handlers for: QdrantConnectionError → 503, EmbeddingError → 503
  - Generic exception handler → 500 internal_error
  - All errors include trace_id
  - **Accept**: Service errors return proper error codes

- [X] T024 [P] Configure structured logging in `backend/logging_config.py`
  - Use structlog for JSON logging
  - Log: trace_id, query_hash (privacy), mode, latency_ms, chunks_retrieved, sources_cited
  - Configure log levels from LOG_LEVEL env var
  - **Accept**: Request logs include all metrics fields

- [X] T025 Create database module for conversations in `backend/database.py`
  - Implement async connection pool with asyncpg
  - Function: `save_conversation(session_id, query, response, sources, mode, ...)`
  - Function: `get_conversation_history(session_id, limit=10)` with MAX_CONVERSATION_HISTORY enforcement
  - Function: `truncate_context_to_token_limit(messages, max_tokens=8000)` for context window management
  - Create table if not exists on startup (schema per spec.md lines 223-241)
  - **Accept**: Conversations persist to Neon Postgres; history retrieval respects limits
  - **Implemented**: Full asyncpg module with connection pool, schema auto-creation, and all required functions

- [X] T026 Implement entry point in `backend/server.py` (renamed from main.py to avoid conflict with embedding pipeline)
  - Import app from api
  - Run uvicorn with host="0.0.0.0", port=8000
  - **Accept**: `python server.py` starts server on port 8000

- [X] T027 [P] Write end-to-end smoke test in `backend/tests/test_api_smoke.py`
  - Test general mode: query → answer + sources
  - Test selected-text mode: query + selection → grounded answer
  - Test streaming: verify SSE event sequence
  - Test health endpoint: all services up
  - **Accept**: `pytest tests/test_api_smoke.py` passes (31/31 tests pass)

**Checkpoint**: All features complete and tested

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - US1 and US2 can proceed in parallel after Foundation
  - US3 depends on US1 (needs run_agent working)
  - US4 can proceed after US1
- **Polish (Phase 7)**: Can start after US1, parallelize with US2-US4

### User Story Dependencies

- **User Story 1 (P1)**: Foundation complete → no dependencies on other stories
- **User Story 2 (P1)**: Foundation complete → no dependencies (parallel with US1)
- **User Story 3 (P2)**: US1 complete (needs run_agent base) → extends with streaming
- **User Story 4 (P3)**: US1 complete (needs search_documentation) → adds parameters

### Within Each User Story

- Models/schemas before implementation
- Core function before endpoint
- Endpoint before validation/error handling

### Critical Path

```
T001 → T005 → T006 → T007 → T008 → T009 → T011 → T017 → T018 → T021
```

### Parallel Opportunities

**After T001 (Setup)**:
```
T002, T003, T004 can run in parallel (different files)
```

**After T007 (Foundation)**:
```
T008 (US1) and T013 (US2) can start in parallel
```

**During Polish**:
```
T021, T022, T023, T024, T027 can all run in parallel (different files)
```

---

## Parallel Example: Phase 1 Setup

```bash
# After T001 completes, launch in parallel:
Task: T002 - Create .env.example
Task: T003 - Create api_models.py
Task: T004 - Create api.py scaffold
```

## Parallel Example: User Stories 1 & 2

```bash
# After Foundation (T007) completes, launch in parallel:
Task: T008 [US1] - Implement search_documentation tool
Task: T013 [US2] - Implement selected-text agent config

# These work on same file (agent.py) but different functions
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T007)
3. Complete Phase 3: User Story 1 (T008-T012)
4. **STOP and VALIDATE**: Test `/chat` endpoint with general mode
5. Deploy/demo if ready - core Q&A with citations working

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add User Story 1 → Test `/chat` general → Deploy (MVP!)
3. Add User Story 2 → Test selected-text → Deploy
4. Add User Story 3 → Test streaming → Deploy
5. Add User Story 4 → Test parameters → Deploy
6. Polish phase → Production ready

### Task Counts

| Phase | Tasks | IDs |
|-------|-------|-----|
| Setup | 4 | T001-T004 |
| Foundational | 3 | T005-T007 |
| US1: General Q&A | 5 | T008-T012 |
| US2: Selected-Text | 3 | T013-T015 |
| US3: Streaming | 3 | T016-T018 |
| US4: Parameters | 2 | T019-T020 |
| Polish | 7 | T021-T027 |
| **Total** | **27** | |

---

## Notes

- [P] tasks = different files or functions, no blocking dependencies
- [Story] label maps task to user story for traceability
- Each user story independently testable after completion
- Commit after each task
- Stop at checkpoints to validate
- Spec-2 modules (retrieval.py, models.py, errors.py) are READ-ONLY - import, don't modify
