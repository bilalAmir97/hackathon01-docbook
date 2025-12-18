# Implementation Plan: RAG Agent API

**Branch**: `006-rag-agent-api` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-rag-agent-api/spec.md`

## Summary

Build a FastAPI backend that hosts an OpenAI Agents SDK agent powered by Gemini (via OpenAI-compatible endpoint). The agent answers questions about the book by retrieving relevant chunks from Qdrant (via Spec-2 retrieval interface) and grounds responses in those sources with proper citations. Supports two modes: general retrieval and selected-text-only mode for focused Q&A.

## Technical Context

| Aspect | Value |
|--------|-------|
| **Language/Version** | Python 3.11 (managed via `uv`, matches Spec-1/Spec-2) |
| **Primary Dependencies** | `openai-agents` (agent SDK), `fastapi`, `uvicorn`, `sse-starlette`, `asyncpg`, `pydantic>=2.0` |
| **Inherited Dependencies** | `cohere`, `qdrant-client`, `python-dotenv` (from Spec-2) |
| **LLM Backend** | Gemini API via OpenAI-compatible endpoint |
| **Storage** | Qdrant Cloud (collection: `rag_embedding`), Neon Postgres (conversations) |
| **Testing** | pytest, pytest-asyncio, httpx (test client) |
| **Performance Goals** | <3s p95 total latency, <1s first-chunk streaming |
| **Constraints** | Use Spec-2 retrieval as library import, disable OpenAI tracing |

## Constitution Check

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Embodied Intelligence | N/A | API infrastructure, not robotics content |
| II. Simulation-First | N/A | Backend service, no simulation |
| III. Technical Rigor | PASS | Type-safe Python, tested contracts |
| IV. Clarity & Structure | PASS | Modular FastAPI design |
| V. Accessibility & Readability | PASS | OpenAPI docs, typed schemas |
| VI. Verification & Citation | PASS | All responses cite sources, no hallucination |

**Gate Result**: PASS

---

## Architecture Overview

### System Architecture Diagram

```
+-----------------------------------------------------------------------------+
|                              Client Request                                  |
|                    POST /chat or POST /chat/stream                          |
+------------------------------------+----------------------------------------+
                                     |
                                     v
+-----------------------------------------------------------------------------+
|                           FastAPI Layer (api.py)                            |
|  +------------------+  +------------------+  +---------------------------+  |
|  |  ChatRequest     |  |  Input           |  |  Request Routing          |  |
|  |  Validation      |  |  Sanitization    |  |  (mode: general/selected) |  |
|  +------------------+  +------------------+  +---------------------------+  |
+------------------------------------+----------------------------------------+
                                     |
          +--------------------------+-------------------------+
          |                                                    |
          v                                                    v
+----------------------+                         +----------------------+
|   General Mode       |                         |  Selected-Text Mode  |
|   (Qdrant RAG)       |                         |  (Local Context)     |
+-----------+----------+                         +-----------+----------+
            |                                                |
            v                                                v
+-----------------------------------------------------------------------------+
|                    OpenAI Agents SDK Agent (agent.py)                       |
|  +-----------------------------------------------------------------------+  |
|  |  Agent Configuration                                                  |  |
|  |  - name: "RAG Assistant"                                              |  |
|  |  - model: Gemini via LiteLLM                                          |  |
|  |  - tools: [search_documentation] (general) or [] (selected-text)      |  |
|  |  - system_prompt: Grounding + citation instructions                   |  |
|  +-----------------------------------------------------------------------+  |
|  +-----------------------------------------------------------------------+  |
|  |  @function_tool search_documentation(query, top_k)                    |  |
|  |  - Wraps Spec-2 retrieve() function                                   |  |
|  |  - Returns structured chunks with metadata                            |  |
|  +-----------------------------------------------------------------------+  |
+------------------------------------+----------------------------------------+
                                     |
          +--------------------------+-------------------------+
          |                          |                         |
          v                          v                         v
+------------------+      +------------------+      +---------------------+
|  Qdrant Cloud    |      |   Gemini API     |      |   Cohere API        |
|  (rag_embedding) |      |   (generation)   |      |   (query embedding) |
+------------------+      +------------------+      +---------------------+
                                     |
                                     v
+-----------------------------------------------------------------------------+
|                        Response Processing                                   |
|  +------------------+  +------------------+  +---------------------------+  |
|  |  Citation        |  |  Conversation    |  |  Response Formatting      |  |
|  |  Extraction      |  |  Storage (Neon)  |  |  (JSON or SSE stream)     |  |
|  +------------------+  +------------------+  +---------------------------+  |
+------------------------------------+----------------------------------------+
                                     |
                                     v
+-----------------------------------------------------------------------------+
|                           ChatResponse                                       |
|  { answer, sources: [SourceCitation...], metadata: {...} }                  |
+-----------------------------------------------------------------------------+
```

### Data Flow: General Mode

```
1. Client -> POST /chat { query, mode: "general", top_k: 5 }
2. FastAPI validates request, generates trace_id
3. Agent invoked with Runner.run() or Runner.run_streamed()
4. Agent calls search_documentation tool:
   a. Tool wraps Spec-2 retrieve(RetrievalRequest)
   b. Cohere embeds query (input_type="search_query")
   c. Qdrant searches rag_embedding collection
   d. Returns ranked ChunkResult[] with metadata
5. Agent generates answer grounded in retrieved chunks
6. Response processor extracts citations from agent output
7. Conversation stored in Neon Postgres
8. ChatResponse returned to client
```

### Data Flow: Selected-Text Mode

```
1. Client -> POST /chat { query, mode: "selected_text", selected_text: "..." }
2. FastAPI validates request, verifies selected_text present
3. Agent invoked WITHOUT search_documentation tool
4. Agent receives selected_text in system prompt context
5. Agent generates answer grounded ONLY in selected_text
6. If answer not in selection -> "not found in selection" response
7. Citations reference "Selected text" with char offsets
8. Conversation stored in Neon Postgres
9. ChatResponse returned to client
```

---

## Project Structure

### Documentation (this feature)

```
specs/006-rag-agent-api/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Research findings
├── data-model.md        # Schema definitions
├── quickstart.md        # Implementation guide
├── contracts/
│   └── api-contracts.md # OpenAPI-style contracts
└── checklists/
    └── requirements.md  # Requirement tracking
```

### Source Code (repository root)

```
backend/
├── retrieval.py         # EXISTING (Spec-2): retrieve(), embed_query()
├── models.py            # EXISTING (Spec-2): ChunkResult, SearchResult
├── config.py            # EXISTING (Spec-2): QDRANT_COLLECTION, etc.
├── errors.py            # EXISTING (Spec-2): Error classes
├── agent.py             # NEW: Agent + @function_tool search_documentation
├── api.py               # NEW: FastAPI endpoints (/chat, /health)
├── api_models.py        # NEW: Pydantic request/response models
├── database.py          # NEW: Neon Postgres conversation storage
├── main.py              # MODIFIED: FastAPI app entry point
├── logging_config.py    # NEW: Structured logging setup
├── pyproject.toml       # MODIFIED: Add new dependencies
└── tests/
    ├── test_agent.py    # NEW: Agent unit tests
    ├── test_api.py      # NEW: API endpoint tests
    ├── test_database.py # NEW: Database tests
    └── integration/
        └── test_e2e_chat.py  # NEW: E2E chat tests
```

---

## Key Design Decisions

### Decision 1: Streaming vs Non-Streaming

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| Non-streaming only | Simpler implementation | Slow perceived response | |
| Streaming only | Fast first-token | Complex citation handling | |
| **Both endpoints** | Flexibility for clients | Two code paths | **Selected** |

**Rationale**: Support both `/chat` (non-streaming) and `/chat/stream` (SSE). Citations sent at end of stream.

### Decision 2: Tool Interface Design

**Selected Design**: Single `search_documentation` tool with structured input/output.

```python
@function_tool
def search_documentation(
    query: Annotated[str, "Search query for documentation"],
    top_k: Annotated[int, "Number of results (1-20)"] = 5,
) -> list[dict]:
    """Search the documentation for relevant content.

    Returns chunks with source_url, page_title, section_heading, chunk_text.
    """
```

**Rationale**:
- Single tool keeps agent interactions simple
- Structured return enables reliable citation extraction
- top_k parameter gives agent control over retrieval depth

### Decision 3: Citation Payload Shape

**Selected Design**: Structured citations with position offsets for selected-text mode.

```python
class SourceCitation(BaseModel):
    source_url: str           # URL or "selected_text"
    page_title: str           # Title or "User Selection"
    section_heading: str      # Section or query-derived
    chunk_text: str           # Snippet (max 500 chars)
    relevance_score: float    # 0.0-1.0

class SelectedTextCitation(SourceCitation):
    char_start: int           # Start offset in selected_text
    char_end: int             # End offset (exclusive)
```

**Rationale**: Enables client-side highlighting of cited portions in selected-text mode.

### Decision 4: Selected-Text-Only Enforcement

**Strategy**: Agent configuration swap, NOT runtime filtering.

```python
# General mode agent
general_agent = Agent(
    tools=[search_documentation],
    instructions=GENERAL_INSTRUCTIONS,
)

# Selected-text mode agent (NO tools)
selected_text_agent = Agent(
    tools=[],  # Empty - no retrieval capability
    instructions=SELECTED_TEXT_INSTRUCTIONS,
)
```

**Rationale**:
- Removing tools prevents accidental Qdrant calls
- System prompt explicitly constrains grounding to selection
- Zero risk of information leakage from external sources

### Decision 5: Error/Fallback Behavior

**Taxonomy**:

| Error Code | HTTP | Condition | Fallback |
|------------|------|-----------|----------|
| `validation_error` | 400 | Invalid input | Return validation details |
| `retrieval_unavailable` | 503 | Qdrant down | "Service temporarily unavailable" |
| `embedding_failed` | 503 | Cohere down | Retry 2x, then error |
| `agent_unavailable` | 503 | Gemini down | Retry 2x, then error |
| `no_relevant_chunks` | 200 | Empty retrieval | "No relevant information found" |
| `not_in_selection` | 200 | Selected-text miss | "Selection doesn't contain..." |

**"No Hallucinated Citations" Rule**:
- Every `source_url` in response MUST exist in retrieved chunks
- Post-processing validation before returning response
- Log warning and strip invalid citations

---

## OpenAI Agents SDK Integration

### Agent Configuration (from Context7 Research)

Based on the OpenAI Agents SDK documentation, here is the recommended integration pattern:

```python
from agents import Agent, function_tool, Runner, set_tracing_disabled
from agents.extensions.models.litellm_model import LitellmModel
from typing import Annotated
import os

# Disable tracing (Gemini backend, no OpenAI tracing needed)
set_tracing_disabled(disabled=True)

# System prompts
GENERAL_SYSTEM_PROMPT = """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.

CRITICAL RULES:
1. ONLY use information from the search_documentation tool results
2. NEVER make claims not supported by retrieved content
3. ALWAYS cite sources for factual claims using [Source: page_title]
4. If information is not found, say "I couldn't find information about that in the documentation"
5. When citing multiple sources, list each one

FORMAT:
- Answer the question clearly and concisely
- Include inline citations like [Source: Page Title]
- If relevant, suggest related topics the user might explore
"""

SELECTED_TEXT_SYSTEM_PROMPT = """You are analyzing a specific text selection provided by the user.

CRITICAL RULES:
1. ONLY answer based on the provided selected_text below
2. Do NOT use any external knowledge or make assumptions
3. If the answer is NOT in the selected text, respond:
   "The provided selection does not contain information about [topic]. Would you like me to search the full documentation?"
4. Cite specific parts of the selection in your answer

The user's selected text is:
---
{selected_text}
---
"""
```

### Tool Definition

```python
from typing import Annotated
from agents import function_tool

# Import from Spec-2
from retrieval import retrieve
from models import RetrievalRequest

@function_tool
def search_documentation(
    query: Annotated[str, "Search query for relevant documentation"],
    top_k: Annotated[int, "Number of results to retrieve (1-20)"] = 5,
) -> list[dict]:
    """Search the Physical AI & Humanoid Robotics documentation.

    Returns relevant text chunks with source metadata for citation.
    Each result contains: source_url, page_title, section_heading, chunk_text, score.
    """
    request = RetrievalRequest(query=query, top_k=min(top_k, 20))
    result = retrieve(request)

    return [
        {
            "source_url": chunk.source_url,
            "page_title": chunk.page_title,
            "section_heading": chunk.section_heading or "Introduction",
            "chunk_text": chunk.chunk_text[:2000],  # Truncate for context window
            "relevance_score": chunk.score,
        }
        for chunk in result.results
    ]
```

### Runner Integration

```python
import asyncio
from agents import Agent, Runner
from agents.extensions.models.litellm_model import LitellmModel

async def run_agent(
    query: str,
    mode: str,
    selected_text: str | None = None,
    top_k: int = 5,
    session_id: str | None = None,
) -> tuple[str, list[dict]]:
    """Run the agent and return (answer, sources).

    Args:
        query: User's question
        mode: "general" or "selected_text"
        selected_text: Required for selected_text mode
        top_k: Number of retrieval results
        session_id: For conversation context

    Returns:
        Tuple of (answer_text, list of source dicts)
    """
    if mode == "selected_text":
        # No tools - constrained to selection only
        agent = Agent(
            name="Selection Assistant",
            instructions=SELECTED_TEXT_SYSTEM_PROMPT.format(
                selected_text=selected_text
            ),
            tools=[],
            model=LitellmModel(
                model="gemini/gemini-2.0-flash",
                api_key=os.environ["GEMINI_API_KEY"],
            ),
        )
        sources = [{
            "source_url": "selected_text",
            "page_title": "User Selection",
            "section_heading": "Selected Text",
            "chunk_text": selected_text[:500],
            "relevance_score": 1.0,
        }]
    else:
        # Full RAG agent with retrieval tool
        agent = Agent(
            name="RAG Assistant",
            instructions=GENERAL_SYSTEM_PROMPT,
            tools=[search_documentation],
            model=LitellmModel(
                model="gemini/gemini-2.0-flash",
                api_key=os.environ["GEMINI_API_KEY"],
            ),
        )
        sources = []  # Will be populated from tool calls

    result = await Runner.run(agent, input=query)

    # Extract sources from tool calls if general mode
    if mode == "general":
        sources = extract_sources_from_result(result)

    return result.final_output, sources


def extract_sources_from_result(result) -> list[dict]:
    """Extract sources from agent run result."""
    sources = []
    # Iterate through run items to find tool outputs
    for item in result.run_items:
        if hasattr(item, 'output') and isinstance(item.output, list):
            sources.extend(item.output)
    return sources
```

### Streaming Implementation

```python
from openai.types.responses import ResponseTextDeltaEvent

async def run_agent_streamed(
    query: str,
    mode: str,
    selected_text: str | None = None,
    top_k: int = 5,
):
    """Run agent with streaming and yield events."""
    # Configure agent based on mode
    if mode == "selected_text":
        agent = Agent(
            name="Selection Assistant",
            instructions=SELECTED_TEXT_SYSTEM_PROMPT.format(
                selected_text=selected_text
            ),
            tools=[],
            model=LitellmModel(
                model="gemini/gemini-2.0-flash",
                api_key=os.environ["GEMINI_API_KEY"],
            ),
        )
    else:
        agent = Agent(
            name="RAG Assistant",
            instructions=GENERAL_SYSTEM_PROMPT,
            tools=[search_documentation],
            model=LitellmModel(
                model="gemini/gemini-2.0-flash",
                api_key=os.environ["GEMINI_API_KEY"],
            ),
        )

    sources = []
    result = Runner.run_streamed(agent, input=query)

    async for event in result.stream_events():
        if event.type == "raw_response_event":
            if isinstance(event.data, ResponseTextDeltaEvent):
                yield {"type": "chunk", "content": event.data.delta}
        elif event.type == "run_item_stream_event":
            if event.item.type == "tool_call_output_item":
                # Capture sources from tool output
                if isinstance(event.item.output, list):
                    sources.extend(event.item.output)

    # Final events
    yield {"type": "sources", "sources": sources}
    yield {"type": "done", "metadata": {"model": "gemini-2.0-flash"}}
```

---

## API Contracts

### POST /chat

**Request:**
```json
{
  "query": "What is ROS 2?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "mode": "general",
  "selected_text": null,
  "top_k": 5,
  "score_threshold": null,
  "filters": null
}
```

**Response (200 OK):**
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a robotics middleware framework...",
  "sources": [
    {
      "source_url": "https://book.example.com/ros2/intro",
      "page_title": "Introduction to ROS 2",
      "section_heading": "What is ROS 2?",
      "chunk_text": "ROS 2 is an open-source robotics middleware...",
      "relevance_score": 0.89
    }
  ],
  "mode": "general",
  "metadata": {
    "query_time_ms": 1250.5,
    "chunks_retrieved": 5,
    "model": "gemini-2.0-flash"
  }
}
```

### POST /chat/stream

**Request:** Same as `/chat`

**Response (200 OK, text/event-stream):**
```
data: {"type": "chunk", "content": "ROS 2 is "}
data: {"type": "chunk", "content": "a robotics middleware..."}
data: {"type": "sources", "sources": [...]}
data: {"type": "done", "metadata": {...}}
```

### GET /health

**Response (200 OK):**
```json
{
  "status": "healthy",
  "services": {
    "qdrant": {"status": "up", "latency_ms": 45.2},
    "cohere": {"status": "up", "latency_ms": 120.5},
    "gemini": {"status": "up", "latency_ms": 200.1},
    "database": {"status": "up", "latency_ms": 30.0}
  },
  "timestamp": "2025-12-16T10:30:00Z"
}
```

### Error Response Format

```json
{
  "error_code": "retrieval_unavailable",
  "message": "Qdrant service is temporarily unavailable",
  "details": {"retry_after": 30},
  "trace_id": "abc123-def456"
}
```

---

## Logging/Tracing Design

### Structured Logging Format

```python
import structlog
import hashlib

logger = structlog.get_logger()

# Example log entry
logger.info(
    "chat_request_processed",
    trace_id="abc123",
    query_hash=hashlib.md5(query.encode()).hexdigest()[:8],  # Privacy
    mode=mode,
    latency_ms=1250.5,
    chunks_retrieved=5,
    sources_cited=3,
)
```

### Log Levels

| Level | Usage |
|-------|-------|
| DEBUG | Tool calls, embedding details, full responses (dev only) |
| INFO | Request/response summaries, latencies |
| WARNING | Empty retrievals, stripped citations |
| ERROR | Service failures, validation errors |

### Key Metrics to Log

```python
from dataclasses import dataclass
from datetime import datetime

@dataclass
class RequestMetrics:
    trace_id: str               # UUID4
    timestamp: datetime
    query_fingerprint: str      # Hashed query (privacy)
    mode: str
    retrieval_ms: float | None
    embedding_ms: float | None
    llm_ms: float
    total_ms: float
    chunks_retrieved: int
    sources_cited: int
    error_code: str | None
```

---

## Testing Strategy

### Unit Tests (test_agent.py)

```python
import pytest

@pytest.mark.asyncio
async def test_search_documentation_tool():
    """Tool returns valid chunk format."""
    result = search_documentation("ROS 2", top_k=3)
    assert isinstance(result, list)
    assert len(result) <= 3
    assert all("source_url" in r for r in result)
    assert all("chunk_text" in r for r in result)

@pytest.mark.asyncio
async def test_selected_text_mode_no_tool_calls():
    """Selected-text mode agent has no tools."""
    agent = Agent(
        name="Selection Assistant",
        instructions=SELECTED_TEXT_SYSTEM_PROMPT.format(selected_text="Test"),
        tools=[],
    )
    assert len(agent.tools) == 0
```

### API Tests (test_api.py)

```python
from fastapi.testclient import TestClient

def test_chat_endpoint_validation():
    """Invalid requests return 400."""
    response = client.post("/chat", json={"query": ""})
    assert response.status_code == 400
    assert response.json()["error_code"] == "validation_error"

def test_chat_general_mode():
    """General mode returns answer with sources."""
    response = client.post("/chat", json={
        "query": "What is ROS 2?",
        "mode": "general"
    })
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert len(data["sources"]) > 0

def test_chat_selected_text_not_found():
    """Selected-text mode returns not-found message."""
    response = client.post("/chat", json={
        "query": "What is quantum computing?",
        "mode": "selected_text",
        "selected_text": "ROS 2 is a robotics framework."
    })
    assert response.status_code == 200
    data = response.json()
    assert "does not contain" in data["answer"].lower()
```

### Integration Tests (test_e2e_chat.py)

```python
# Deterministic test prompts with expected sources
GOLDEN_TESTS = [
    {
        "query": "What is ROS 2?",
        "expected_source_url_contains": "ros2",
    },
    {
        "query": "How do I launch Gazebo?",
        "expected_source_url_contains": "gazebo",
    },
]

@pytest.mark.asyncio
async def test_golden_queries():
    """Verify expected sources are retrieved."""
    for test in GOLDEN_TESTS:
        response = await client.post("/chat", json={
            "query": test["query"],
            "mode": "general"
        })
        sources = response.json()["sources"]
        assert any(
            test["expected_source_url_contains"] in s["source_url"]
            for s in sources
        ), f"Expected source not found for: {test['query']}"
```

### Test Coverage Targets

| Category | Tests | Priority |
|----------|-------|----------|
| Endpoint validation | 10 | P1 |
| Agent tool execution | 8 | P1 |
| Selected-text mode | 6 | P1 |
| Streaming responses | 5 | P2 |
| Error handling | 10 | P1 |
| Health checks | 4 | P2 |
| Database operations | 6 | P3 |
| **Total** | **49** | |

---

## Non-Functional Requirements

| Requirement | Target | Measurement |
|-------------|--------|-------------|
| p95 total latency | < 3000ms | Time from request to response |
| p95 first-chunk latency | < 1000ms | Time to first SSE chunk |
| Concurrent requests | 10 | Simultaneous connections |
| Citation accuracy | 100% | All cited URLs from retrieval |
| Grounding compliance | 100% | No claims without sources |
| Health check latency | < 500ms | GET /health response time |

---

## Implementation Phases

### Phase 1: Core Agent (Days 1-2)
- [ ] Create `agent.py` with `@function_tool search_documentation`
- [ ] Integrate with Spec-2 `retrieve()` function
- [ ] Configure Gemini via LiteLLM
- [ ] Implement `run_agent()` for non-streaming

### Phase 2: FastAPI Endpoints (Days 2-3)
- [ ] Create `api_models.py` with Pydantic schemas
- [ ] Implement `POST /chat` endpoint
- [ ] Implement `GET /health` endpoint
- [ ] Add request validation and error handling

### Phase 3: Selected-Text Mode (Day 3)
- [ ] Implement agent configuration swap
- [ ] Add selected-text system prompt
- [ ] Implement citation extraction for selection

### Phase 4: Streaming (Day 4)
- [ ] Implement `POST /chat/stream` endpoint
- [ ] Add SSE event formatting
- [ ] Handle streaming citation delivery

### Phase 5: Database & Polish (Day 5)
- [ ] Implement Neon Postgres conversation storage
- [ ] Add structured logging
- [ ] Write integration tests
- [ ] Document API with OpenAPI

---

## ADR Candidates

The following decisions warrant ADR documentation:

1. **ADR-0006: Spec-2/Spec-3 Integration Pattern** - Library import vs HTTP call (ALREADY EXISTS)
2. **ADR-0007: Selected-Text Enforcement via Agent Configuration** - Why tools are removed vs runtime filtering
3. **ADR-0008: Gemini via OpenAI-Compatible Endpoint** - Using LiteLLM vs native Gemini SDK

---

## Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| `openai-agents` | latest | Agent SDK |
| `fastapi` | ^0.109 | Web framework |
| `uvicorn` | ^0.27 | ASGI server |
| `sse-starlette` | ^1.8 | SSE streaming |
| `asyncpg` | ^0.29 | Postgres async driver |
| `pydantic` | ^2.0 | Schema validation |
| `structlog` | ^24.1 | Structured logging |

---

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| `GEMINI_API_KEY` | Yes | Gemini API key |
| `COHERE_API_KEY` | Yes | Cohere embeddings |
| `QDRANT_URL` | Yes | Qdrant Cloud URL |
| `QDRANT_API_KEY` | Yes | Qdrant API key |
| `DATABASE_URL` | Yes | Neon Postgres URL |
| `GEMINI_MODEL` | No | Model name (default: gemini-2.0-flash) |
| `LOG_LEVEL` | No | Logging level (default: INFO) |

---

## Risk Analysis

| Risk | Impact | Mitigation |
|------|--------|------------|
| Gemini API latency spikes | Slow responses | Implement timeouts, streaming |
| Agent hallucination | Incorrect citations | System prompt constraints, post-validation |
| Selected-text bypass | Wrong grounding | Remove tools entirely in selected mode |
| Qdrant downtime | Service unavailable | Health checks, graceful error messages |

---

## Acceptance Checklist

| # | Criterion | Status |
|---|-----------|--------|
| 1 | Agent answers questions using Qdrant retrieval | [ ] |
| 2 | All citations reference valid retrieved sources | [ ] |
| 3 | Selected-text mode works without Qdrant calls | [ ] |
| 4 | Streaming endpoint delivers incremental chunks | [ ] |
| 5 | Health endpoint checks all services | [ ] |
| 6 | Error responses follow taxonomy | [ ] |
| 7 | Logging includes trace IDs and latencies | [ ] |
| 8 | Tests cover critical paths | [ ] |
