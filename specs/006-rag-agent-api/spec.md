# Feature Specification: RAG Agent API

**Feature Branch**: `006-rag-agent-api`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "Build a FastAPI backend that hosts an OpenAI Agents SDK agent which answers questions about the book by retrieving relevant chunks from Qdrant and grounding responses in those sources."

---

## Overview

This feature builds a FastAPI backend that hosts an intelligent agent using the OpenAI Agents SDK. The agent answers questions about the book by retrieving relevant chunks from the Qdrant vector database (populated by Spec-1, queried via Spec-2 retrieval interface) and grounding responses in those sources with proper citations.

**Target Users**: Backend developers implementing an agentic RAG layer on top of the existing Qdrant retrieval pipeline.

**Core Value**: Enable natural language Q&A over the book content with source attribution, providing users with accurate answers grounded in the actual documentation.

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - General Question Answering with Citations (Priority: P1)

A user asks a general question about the book content. The agent retrieves relevant chunks from Qdrant, synthesizes an answer, and provides citations (source URLs, page titles, section references) so the user can verify or explore further.

**Why this priority**: This is the core value proposition - answering questions with verifiable sources.

**Independent Test**: Can be fully tested by sending a question via the API and verifying the response includes an answer plus source citations.

**Acceptance Scenarios**:

1. **Given** a user asks "What is ROS 2?", **When** the agent processes the query, **Then** it returns an answer synthesized from relevant chunks plus citations (source_url, page_title, section_heading for each source used).
2. **Given** the agent retrieves multiple relevant chunks, **When** generating a response, **Then** it cites all chunks that contributed to the answer (not just the top-1).
3. **Given** a question about content that exists in the book, **When** the agent responds, **Then** the answer is factually consistent with the retrieved chunks (no hallucination).
4. **Given** a question about content not in the book, **When** the agent processes the query, **Then** it responds with "I don't have information about that in the available documentation" (or similar safe fallback).

---

### User Story 2 - Selected-Text-Only Mode (Priority: P1)

A user provides selected text (e.g., a specific passage they're viewing) and asks a question. The agent restricts its answers to ONLY the provided selection, ignoring the broader Qdrant index. If the answer isn't in the selection, it explicitly says so.

**Why this priority**: Critical for focused Q&A where users want answers scoped to specific content they're reading.

**Independent Test**: Can be tested by providing selected text plus a query and verifying the response is grounded only in that text.

**Acceptance Scenarios**:

1. **Given** a user provides selected text and asks a question, **When** the answer exists in the selection, **Then** the agent responds using only that text and cites "Selected text" as the source.
2. **Given** a user provides selected text and asks a question, **When** the answer is NOT in the selection (even if it exists elsewhere in the book), **Then** the agent responds with "The provided selection does not contain information about [topic]. Would you like me to search the full documentation?"
3. **Given** selected-text mode is active, **When** the agent generates a response, **Then** it does NOT query Qdrant or use any content outside the provided selection.
4. **Given** empty selected text is provided, **When** processing the request, **Then** the system returns an error indicating selection is required for this mode.

---

### User Story 3 - Chat API with Response Streaming (Priority: P2)

A developer integrates the agent via an HTTP API endpoint that supports streaming responses, allowing real-time display of the agent's answer as it's generated.

**Why this priority**: Streaming improves perceived performance for longer answers and enables real-time UI updates.

**Independent Test**: Can be tested by calling the streaming endpoint and verifying chunks arrive progressively.

**Acceptance Scenarios**:

1. **Given** a developer calls the streaming chat endpoint, **When** the agent generates a response, **Then** the response streams as Server-Sent Events (SSE) with partial content chunks.
2. **Given** streaming is enabled, **When** the response completes, **Then** the final event includes the complete citations/sources object.
3. **Given** a developer prefers non-streaming, **When** they call the standard chat endpoint, **Then** they receive the complete response in a single JSON payload.

---

### User Story 4 - Configurable Retrieval Parameters (Priority: P3)

A developer configures retrieval behavior (top-k, score threshold, metadata filters) per request to tune relevance for different use cases.

**Why this priority**: Flexibility for different query types and integration scenarios.

**Independent Test**: Can be tested by varying parameters and verifying retrieval behavior changes accordingly.

**Acceptance Scenarios**:

1. **Given** a request with `top_k=10`, **When** retrieval occurs, **Then** up to 10 chunks are retrieved and considered for the answer.
2. **Given** a request with `score_threshold=0.7`, **When** retrieval occurs, **Then** only chunks with similarity >= 0.7 are used.
3. **Given** a request with metadata filters (e.g., `source_url` filter), **When** retrieval occurs, **Then** only chunks matching the filter criteria are considered.

---

### Edge Cases

| Scenario | Expected Behavior |
|----------|-------------------|
| Qdrant unreachable | Return error response with `error_code: "retrieval_unavailable"` and user-friendly message |
| Cohere API failure | Return error response with `error_code: "embedding_failed"` |
| Gemini API failure | Return error response with `error_code: "agent_unavailable"` |
| No relevant chunks found | Agent responds: "I couldn't find relevant information in the documentation for your question." |
| Query too long (>2000 chars) | Return 400 error with validation message |
| Selected text too long (>10000 chars) | Return 400 error suggesting user shorten selection |
| Empty query | Return 400 error with validation message |
| Malformed JSON request | Return 400 error with parsing details |
| Rate limit exceeded | Return 429 error with `retry_after` header |

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST expose a `/chat` endpoint that accepts a natural language query and returns an agent-generated answer with citations.
- **FR-002**: System MUST expose a `/chat/stream` endpoint for streaming responses via Server-Sent Events (SSE).
- **FR-003**: System MUST integrate with OpenAI Agents SDK using Gemini API via OpenAI-compatible endpoint (`https://generativelanguage.googleapis.com/v1beta/openai/`) to power the conversational agent.
- **FR-004**: System MUST provide the agent with a retrieval tool that queries Qdrant (using Spec-2 retrieval interface) and returns relevant chunks.
- **FR-005**: System MUST include source citations in every response, containing: source_url, page_title, section_heading, and chunk_text snippet for each cited source.
- **FR-006**: System MUST support "selected-text-only" mode via a request parameter that restricts grounding to user-provided text only.
- **FR-007**: System MUST enforce strict grounding in selected-text mode - if information isn't in the selection, respond with a clear "not found in selection" message.
- **FR-008**: System MUST support configurable retrieval parameters per request: top_k (default 5, range 1-20), score_threshold (optional), and metadata filters.
- **FR-009**: System MUST provide a `/health` endpoint that verifies connectivity to Qdrant, Cohere, Gemini, and Neon Postgres services.
- **FR-010**: System MUST handle all external service failures gracefully with typed error responses and appropriate HTTP status codes.
- **FR-011**: System MUST NOT hallucinate information - all factual claims must be traceable to retrieved chunks or provided selection.
- **FR-012**: System MUST expose an OpenAPI/Swagger documentation endpoint at `/docs`.
- **FR-013**: System MUST store conversation history in Neon Postgres with session-based grouping.
- **FR-014**: System MUST support multi-turn chat by including previous conversation context when generating responses.
- **FR-015**: System MUST accept a `session_id` parameter (client-generated UUID) to group conversations.

### API Contracts

#### ChatRequest

```
ChatRequest:
  query: str                              # User's question (1-2000 chars, required)
  session_id: str | None                  # Client-generated UUID for conversation grouping (optional)
  mode: "general" | "selected_text"       # Query mode (default: "general")
  selected_text: str | None               # Required if mode="selected_text" (1-10000 chars)
  top_k: int = 5                          # Number of chunks to retrieve (1-20)
  score_threshold: float | None           # Minimum similarity score (0.0-1.0)
  filters: dict | None                    # Metadata filters for retrieval
    - source_url: str | list[str]
    - page_title: str | list[str]
    - section_heading: str | list[str]
  stream: bool = false                    # Whether to stream response (for /chat endpoint)
```

#### ChatResponse

```
ChatResponse:
  answer: str                             # Agent's synthesized answer
  sources: list[SourceCitation]           # Citations for the answer
  mode: "general" | "selected_text"       # Mode used for this response
  metadata:
    query_time_ms: float                  # Total response time
    chunks_retrieved: int                 # Number of chunks considered
    model: str                            # Agent model used

SourceCitation:
  source_url: str                         # Full URL of source page (or "selected_text" for selected-text mode)
  page_title: str                         # Page title (or "User Selection" for selected-text mode)
  section_heading: str                    # Section heading
  chunk_text: str                         # Relevant text snippet (truncated to 500 chars)
  relevance_score: float                  # Similarity score (0.0-1.0, or 1.0 for selected-text)

# Additional fields for selected-text mode citations (enables client-side highlighting)
SelectedTextCitation (extends SourceCitation):
  char_start: int                         # Start character offset in selected_text (0-indexed)
  char_end: int                           # End character offset in selected_text (exclusive)
  line_start: int | None                  # Start line number (1-indexed, optional)
  line_end: int | None                    # End line number (1-indexed, optional)
```

#### StreamingResponse (SSE)

```
Event types:
  - "chunk": Partial answer text
  - "sources": Citation array (sent at end)
  - "done": Completion signal with final metadata
  - "error": Error event with details

Example events:
  data: {"type": "chunk", "content": "ROS 2 is a "}
  data: {"type": "chunk", "content": "robotics middleware..."}
  data: {"type": "sources", "sources": [...]}
  data: {"type": "done", "metadata": {...}}
```

#### ErrorResponse

```
ErrorResponse:
  error_code: str                         # Machine-readable code
  message: str                            # Human-readable message
  details: dict | None                    # Additional context
  retry_after: int | None                 # Seconds to wait (for rate limits)
```

#### HealthResponse

```
HealthResponse:
  status: "healthy" | "degraded" | "unhealthy"
  services:
    qdrant: ServiceStatus
    cohere: ServiceStatus
    gemini: ServiceStatus                 # Changed from "openai" to "gemini"
    database: ServiceStatus               # Neon Postgres
  timestamp: datetime

ServiceStatus:
  status: "up" | "down" | "degraded"
  latency_ms: float | None
  message: str | None
```

### Key Entities

- **ChatRequest**: A user's question with optional mode, filters, and retrieval parameters.
- **ChatResponse**: The agent's answer with source citations and metadata.
- **SourceCitation**: A reference to a specific chunk used to generate the answer.
- **Agent**: The OpenAI Agents SDK agent configured with retrieval tool and system prompts.
- **RetrievalTool**: A tool available to the agent that queries Qdrant for relevant chunks.
- **Conversation**: A stored exchange in Neon Postgres containing session_id, query, response, sources, and timestamp.

### Conversation Storage Schema (Neon Postgres)

```
Table: conversations
  id: UUID (primary key, auto-generated)
  session_id: str                         # Client-provided session identifier
  query: str                              # User's question
  response: str                           # Agent's answer
  sources: jsonb                          # Array of SourceCitation objects
  mode: str                               # "general" or "selected_text"
  selected_text: str | None               # User-provided text (for selected_text mode)
  chunks_retrieved: int                   # Number of chunks considered
  latency_ms: float                       # Response time in milliseconds
  created_at: timestamp (default: now())  # When the conversation occurred

Indexes:
  - idx_conversations_session_id ON conversations(session_id)
  - idx_conversations_session_created ON conversations(session_id, created_at DESC)
```

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

| ID | Criterion | Threshold | Measurement Method |
|----|-----------|-----------|-------------------|
| SC-001 | End-to-end response time (p95) | < 3 seconds | Time from request to complete response |
| SC-002 | Streaming first-chunk latency (p95) | < 1 second | Time to first SSE event |
| SC-003 | Citation accuracy | 100% | All cited sources appear in retrieved chunks |
| SC-004 | Grounding compliance | 100% | No factual claims without source evidence |
| SC-005 | Selected-text isolation | 100% | Selected-text mode never uses external sources |
| SC-006 | Fallback reliability | 100% | "Not found" responses when content unavailable |
| SC-007 | Health endpoint accuracy | 100% | Correctly reports service status |
| SC-008 | API documentation completeness | 100% | All endpoints documented in OpenAPI spec |
| SC-009 | Error handling coverage | 100% | All error scenarios return typed errors |
| SC-010 | Concurrent request support | 10 requests | Handle 10 simultaneous requests |

---

## Error Taxonomy

| Code | HTTP Status | Condition | Retry? |
|------|-------------|-----------|--------|
| `validation_error` | 400 | Invalid request parameters | No |
| `retrieval_unavailable` | 503 | Qdrant connection failed | Yes (3x) |
| `embedding_failed` | 503 | Cohere API failure | Yes (2x) |
| `agent_unavailable` | 503 | Gemini API failure | Yes (2x) |
| `rate_limited` | 429 | Rate limit exceeded | Yes (backoff) |
| `not_found` | 404 | Endpoint not found | No |
| `internal_error` | 500 | Unexpected server error | No |

---

## Constraints Catalog

### Technical Constraints (from Spec-1 & Spec-2)

| Constraint | Value | Source |
|------------|-------|--------|
| Qdrant collection | `rag_embedding` | Spec-1/Spec-2 |
| Vector dimension | 1024 | Cohere embed-english-v3.0 |
| Query embedding input_type | `search_query` | Spec-2 |
| Retrieval interface | Spec-2 API | Spec-2 |

### Operational Constraints

| Constraint | Value |
|------------|-------|
| Total request timeout | 30 seconds |
| Retrieval timeout | 5 seconds |
| Agent generation timeout | 25 seconds |
| Max concurrent requests | 10 |
| Rate limit | 60 requests/minute per client |

### Input Constraints

| Constraint | Value |
|------------|-------|
| Query min length | 1 character |
| Query max length | 2000 characters |
| Selected text max length | 10000 characters |
| top_k range | 1-20 |
| score_threshold range | 0.0-1.0 |
| session_id format | UUID v4 (36 chars) or None |

### Conversation Context Constraints

| Constraint | Value |
|------------|-------|
| Max conversation history | 10 messages (5 turns) |
| Max context tokens | 8000 tokens |
| Conversation retention | 30 days (configurable) |
| Database connection timeout | 5 seconds |

---

## Agent Configuration

### System Prompt Guidelines

The agent system prompt MUST:
1. Instruct the agent to ONLY use information from retrieved chunks or provided selection
2. Require citation of sources for all factual claims
3. Specify fallback behavior when information is not found
4. Prohibit speculation or knowledge beyond provided context

### Retrieval Tool Definition

The agent MUST have access to a retrieval tool with:
- Name: `search_documentation`
- Parameters: query (string), top_k (int), filters (object)
- Returns: Array of chunks with metadata

### Tracing Configuration

- **Tracing**: Disabled (`set_tracing_disabled(True)`)
- **Rationale**: Using Gemini backend via OpenAI-compatible endpoint; OpenAI tracing requires OpenAI API key which is not available
- **Alternative**: If tracing needed for debugging, use local logging instead

---

## Implementation File Structure

The following file structure integrates with existing Spec-2 modules:

```
backend/
├── retrieval.py   # EXISTING (Spec-2): Core retrieval with retrieve(), embed_query()
├── models.py      # EXISTING (Spec-2): ChunkResult, SearchResult, RetrievalRequest
├── config.py      # EXISTING (Spec-2): Constants (QDRANT_COLLECTION, COHERE_MODEL, etc.)
├── errors.py      # EXISTING (Spec-2): Error classes (EmbeddingError, QdrantConnectionError)
├── agent.py       # NEW (Spec-3): Agent + @function_tool search_documentation
├── api.py         # NEW (Spec-3): FastAPI endpoints (/chat, /chat/stream, /health)
├── database.py    # NEW (Spec-3): Neon Postgres conversation storage
└── main.py        # NEW (Spec-3): FastAPI app entry point
```

### `agent.py` Responsibilities
- Import `retrieve()` from existing `retrieval.py`
- Define `@function_tool search_documentation()` that wraps retrieval
- Configure Gemini via OpenAI-compatible endpoint with `AsyncOpenAI`
- Create `Agent` instance with system prompt and tools
- Export `run_agent()` function for use by `api.py`

### `api.py` Responsibilities
- FastAPI route handlers for `/chat`, `/chat/stream`, `/health`
- Request/response Pydantic models
- SSE streaming implementation
- Import and call agent from `agent.py`

---

## Spec-2 Integration Pattern

### Decision: Library Import (Recommended)

The RAG Agent API integrates with Spec-2's retrieval interface via **direct library import** rather than HTTP calls or reimplementation.

| Pattern | Pros | Cons | Decision |
|---------|------|------|----------|
| **Library Import** | No network overhead, shared types, single deployment | Tighter coupling | ✅ **Selected** |
| HTTP/REST Call | Loose coupling, independent scaling | Latency overhead, error handling complexity | ❌ |
| Reimplementation | Full control | Code duplication, drift risk | ❌ |

### Integration Contract

Spec-3 imports the following from Spec-2 (matching actual module structure):

```python
# Core retrieval function
from retrieval import retrieve, retrieve_with_retry, embed_query

# Data models
from models import RetrievalRequest, SearchResult, ChunkResult

# Error handling
from errors import (
    EmbeddingError,
    QdrantConnectionError,
    QdrantTimeoutError,
    CollectionNotFoundError,
    InvalidFilterError,
)

# Configuration
from config import QDRANT_COLLECTION, COHERE_MODEL
```

### Shared Types Mapping

| Spec-2 Type | Spec-3 Usage |
|-------------|--------------|
| `RetrievalRequest` | Used by `search_documentation` tool internally |
| `ChunkResult` | Mapped to `SourceCitation` in responses |
| `SearchResult` | Provides `results`, `latency_ms` for metadata |
| `retrieve()` | Called within `@function_tool search_documentation()` |

### Module Location

The Spec-2 modules are located in `backend/` directory:
- `backend/retrieval.py` - Core retrieval with `retrieve()`, `embed_query()`
- `backend/models.py` - Data classes (`RetrievalRequest`, `SearchResult`, `ChunkResult`)
- `backend/errors.py` - Exception classes
- `backend/config.py` - Constants

---

## Assumptions

- Spec-1 pipeline has populated the Qdrant collection (`rag_embedding`) with book content.
- Spec-2 retrieval interface is available and functional.
- Gemini API key is available via environment variable (`GEMINI_API_KEY`) for the OpenAI Agents SDK using Gemini backend.
- OpenAI API key is NOT required (using Gemini via OpenAI-compatible endpoint).
- Cohere API key is available for query embedding (reuses Spec-2 configuration).
- Qdrant Cloud credentials are available via environment variables.
- Neon Postgres connection string is available via `DATABASE_URL` environment variable.
- FastAPI is the chosen web framework.
- Python 3.11+ runtime environment.

---

## Out of Scope

- User authentication and authorization (all endpoints are public)
- Rate limiting implementation details (use standard middleware)
- Deployment infrastructure (Docker, Kubernetes, etc.)
- Frontend UI components
- Analytics and usage tracking
- Custom fine-tuning of the agent model

---

## Dependencies

| Dependency | Type | Description |
|------------|------|-------------|
| Spec-1 (004-Spec01-embedding-pipeline) | Feature | Qdrant collection populated with book vectors |
| Spec-2 (005-spec02-retrieval-validation) | Feature | Retrieval interface for querying Qdrant |
| Gemini API | External | Powers the agent via OpenAI-compatible endpoint |
| Cohere API | External | Query embedding generation |
| Qdrant Cloud | External | Vector storage and retrieval |
| Neon Postgres | External | Conversation history storage (serverless) |
| FastAPI | Library | Web framework |
| OpenAI Agents SDK | Library | Agent implementation (using Gemini as backend) |
| asyncpg | Library | Async PostgreSQL driver for Neon Postgres |
| pydantic | Library | Request/response validation |
| sse-starlette | Library | Server-Sent Events for streaming responses |
| python-dotenv | Library | Environment variable loading |

---

## Environment Variables

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `GEMINI_API_KEY` | Yes | Gemini API key for OpenAI-compatible endpoint | `AIza...` |
| `COHERE_API_KEY` | Yes | Cohere API key for embeddings | `...` |
| `QDRANT_URL` | Yes | Qdrant Cloud cluster URL | `https://xyz.cloud.qdrant.io:6333` |
| `QDRANT_API_KEY` | Yes | Qdrant Cloud API key | `...` |
| `QDRANT_COLLECTION_NAME` | No | Collection name (default: `rag_embedding`) | `rag_embedding` |
| `GEMINI_MODEL` | No | Gemini model name (default: `gemini-2.0-flash`) | `gemini-2.0-flash` |
| `GEMINI_BASE_URL` | No | OpenAI-compatible endpoint (default: `https://generativelanguage.googleapis.com/v1beta/openai/`) | `https://generativelanguage.googleapis.com/v1beta/openai/` |
| `DATABASE_URL` | Yes | Neon Postgres connection string | `postgresql://user:pass@ep-xxx.neon.tech/dbname` |
| `API_RATE_LIMIT` | No | Requests per minute (default: 60) | `100` |
| `MAX_CONVERSATION_HISTORY` | No | Max messages to include in context (default: 10) | `10` |
| `CONVERSATION_RETENTION_DAYS` | No | Days to retain conversations (default: 30) | `30` |

---

## Clarifications

### Session 2025-12-16

- Q: Which LLM provider should power the agent? → A: Gemini API via OpenAI-compatible endpoint (`https://generativelanguage.googleapis.com/v1beta/openai/`)
- Q: Should conversation history be stored? → A: Yes, store in Neon Postgres with session ID, enable multi-turn chat
- Q: What file structure for the agent implementation? → A: Hybrid - `agent.py` (agent + @function_tool) imports from existing `retrieval.py`, separate `api.py` for FastAPI routes
- Q: What is the target response time (p95)? → A: 3 seconds (per specify_prompt.md requirement)
- Q: Should OpenAI Agents SDK tracing be enabled? → A: No, disable tracing (`tracing_disabled=True`) since using Gemini backend

---

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| OpenAI API latency spikes | Slow responses, timeouts | Implement timeouts, show streaming progress |
| Agent hallucination | Incorrect information | Strict system prompt, citation requirements |
| Qdrant downtime | Service unavailable | Health checks, graceful degradation messaging |
| Rate limiting by external APIs | Service disruption | Implement backoff, queue requests |
| Selected-text bypass | Incorrect grounding | Strict mode enforcement in code, no Qdrant calls in selected-text mode |
