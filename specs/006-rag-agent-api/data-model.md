# Data Model: RAG Agent API

**Date**: 2025-12-16 | **Branch**: `006-rag-agent-api`

## Request/Response Schemas

### ChatRequest (Pydantic v2)

```python
from pydantic import BaseModel, Field, model_validator
from typing import Literal

class ChatRequest(BaseModel):
    """Request body for /chat and /chat/stream endpoints."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question",
        examples=["What is ROS 2?"],
    )

    session_id: str | None = Field(
        default=None,
        pattern=r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$",
        description="Client-generated UUID for conversation grouping",
    )

    mode: Literal["general", "selected_text"] = Field(
        default="general",
        description="Query mode",
    )

    selected_text: str | None = Field(
        default=None,
        max_length=10000,
        description="User-provided text for selected_text mode",
    )

    top_k: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of chunks to retrieve",
    )

    score_threshold: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description="Minimum similarity score",
    )

    filters: dict[str, str | list[str]] | None = Field(
        default=None,
        description="Metadata filters (source_url, page_title, section_heading)",
    )

    @model_validator(mode='after')
    def validate_selected_text_mode(self):
        if self.mode == "selected_text" and not self.selected_text:
            raise ValueError("selected_text is required when mode is 'selected_text'")
        return self
```

### SourceCitation

```python
class SourceCitation(BaseModel):
    """Citation reference for a source used in the answer."""

    source_url: str = Field(
        description="URL of source page or 'selected_text'",
    )

    page_title: str = Field(
        description="Page title or 'User Selection'",
    )

    section_heading: str = Field(
        description="Section heading within the page",
    )

    chunk_text: str = Field(
        max_length=500,
        description="Relevant text snippet",
    )

    relevance_score: float = Field(
        ge=0.0,
        le=1.0,
        description="Similarity score",
    )


class SelectedTextCitation(SourceCitation):
    """Extended citation with position offsets for selected-text mode."""

    char_start: int = Field(
        ge=0,
        description="Start character offset in selected_text",
    )

    char_end: int = Field(
        ge=0,
        description="End character offset (exclusive)",
    )

    line_start: int | None = Field(
        default=None,
        description="Start line number (1-indexed)",
    )

    line_end: int | None = Field(
        default=None,
        description="End line number (1-indexed)",
    )
```

### ChatResponse

```python
class ResponseMetadata(BaseModel):
    """Metadata about the chat response."""

    query_time_ms: float = Field(
        description="Total response time in milliseconds",
    )

    chunks_retrieved: int = Field(
        description="Number of chunks considered",
    )

    model: str = Field(
        description="LLM model used",
    )


class ChatResponse(BaseModel):
    """Response body for /chat endpoint."""

    answer: str = Field(
        description="Agent's synthesized answer",
    )

    sources: list[SourceCitation] = Field(
        description="Citations for the answer",
    )

    mode: Literal["general", "selected_text"] = Field(
        description="Mode used for this response",
    )

    metadata: ResponseMetadata = Field(
        description="Response metadata",
    )
```

### StreamingEvents

```python
class StreamChunk(BaseModel):
    """Streaming text chunk event."""
    type: Literal["chunk"] = "chunk"
    content: str


class StreamSources(BaseModel):
    """Streaming sources event (sent at end)."""
    type: Literal["sources"] = "sources"
    sources: list[SourceCitation]


class StreamDone(BaseModel):
    """Streaming completion event."""
    type: Literal["done"] = "done"
    metadata: ResponseMetadata


class StreamError(BaseModel):
    """Streaming error event."""
    type: Literal["error"] = "error"
    error_code: str
    message: str
```

### ErrorResponse

```python
class ErrorResponse(BaseModel):
    """Standard error response format."""

    error_code: str = Field(
        description="Machine-readable error code",
        examples=["validation_error", "retrieval_unavailable"],
    )

    message: str = Field(
        description="Human-readable error message",
    )

    details: dict | None = Field(
        default=None,
        description="Additional error context",
    )

    trace_id: str = Field(
        description="Request trace ID for debugging",
    )
```

### HealthResponse

```python
from datetime import datetime

class ServiceStatus(BaseModel):
    """Status of a single service."""

    status: Literal["up", "down", "degraded"]
    latency_ms: float | None = None
    message: str | None = None


class HealthResponse(BaseModel):
    """Response for /health endpoint."""

    status: Literal["healthy", "degraded", "unhealthy"]

    services: dict[str, ServiceStatus] = Field(
        description="Status of each dependent service",
    )

    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
    )
```

---

## Database Schema (Neon Postgres)

### conversations table

```sql
CREATE TABLE IF NOT EXISTS conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(36) NOT NULL,
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    sources JSONB NOT NULL,
    mode VARCHAR(20) NOT NULL,
    selected_text TEXT,
    chunks_retrieved INTEGER NOT NULL,
    latency_ms FLOAT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_conversations_session_id ON conversations(session_id);
CREATE INDEX idx_conversations_session_created ON conversations(session_id, created_at DESC);
```

### Python Model

```python
from dataclasses import dataclass
from datetime import datetime
from typing import Any

@dataclass
class Conversation:
    id: str
    session_id: str
    query: str
    response: str
    sources: list[dict[str, Any]]
    mode: str
    selected_text: str | None
    chunks_retrieved: int
    latency_ms: float
    created_at: datetime
```

---

## Type Mappings

### Spec-2 to Spec-3

| Spec-2 Type | Spec-3 Type | Mapping |
|-------------|-------------|---------|
| `ChunkResult` | `SourceCitation` | Direct field mapping |
| `SearchResult` | Internal only | Used for latency metadata |
| `RetrievalRequest` | Internal only | Created from ChatRequest |

### Conversion Function

```python
def chunk_to_citation(chunk: ChunkResult) -> SourceCitation:
    return SourceCitation(
        source_url=chunk.source_url,
        page_title=chunk.page_title,
        section_heading=chunk.section_heading or "Introduction",
        chunk_text=chunk.chunk_text[:500],
        relevance_score=chunk.score,
    )
```

---

## Entity Relationship Diagram

```
+---------------+       +-------------------+       +------------------+
|  ChatRequest  |       |   Agent Runtime   |       |  ChatResponse    |
+---------------+       +-------------------+       +------------------+
| query         |------>| process query     |------>| answer           |
| mode          |       | call tools        |       | sources[]        |
| selected_text |       | generate answer   |       | mode             |
| top_k         |       +--------+----------+       | metadata         |
| filters       |                |                  +------------------+
+---------------+                |
                                 |
                                 v
                    +------------------------+
                    |  Qdrant (rag_embedding)|
                    +------------------------+
                    | ChunkResult[]          |
                    | - source_url           |
                    | - page_title           |
                    | - section_heading      |
                    | - chunk_text           |
                    | - score                |
                    +------------------------+
                                 |
                                 v
                    +------------------------+
                    |  Neon Postgres         |
                    +------------------------+
                    | Conversation           |
                    | - session_id           |
                    | - query                |
                    | - response             |
                    | - sources (JSONB)      |
                    +------------------------+
```

---

## Validation Rules Summary

| Field | Validation |
|-------|------------|
| query | Required, 1-2000 chars |
| session_id | Optional, UUID v4 format |
| mode | "general" or "selected_text" |
| selected_text | Required if mode="selected_text", max 10000 chars |
| top_k | 1-20, default 5 |
| score_threshold | 0.0-1.0, optional |
| filters | Optional, dict with string/list values |
