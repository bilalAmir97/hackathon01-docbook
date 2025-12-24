"""Pydantic models for RAG Agent API request/response schemas.

This module defines all request and response models for the /chat, /chat/stream,
and /health endpoints. Models use Pydantic v2 with Field validators.
"""

from __future__ import annotations

from datetime import datetime
from typing import Literal

from pydantic import BaseModel, Field, model_validator


# =============================================================================
# Request Models
# =============================================================================


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
        description="Query mode: 'general' uses Qdrant retrieval, 'selected_text' uses provided text only",
    )

    selected_text: str | None = Field(
        default=None,
        max_length=10000,
        description="User-provided text for selected_text mode",
    )

    top_k: int = Field(
        default=10,
        ge=1,
        le=20,
        description="Number of chunks to retrieve",
    )

    score_threshold: float | None = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description="Minimum similarity score for retrieval",
    )

    filters: dict[str, str | list[str]] | None = Field(
        default=None,
        description="Metadata filters (source_url, page_title, section_heading)",
    )

    @model_validator(mode="after")
    def validate_selected_text_mode(self) -> "ChatRequest":
        """Ensure selected_text is provided when mode is 'selected_text'."""
        if self.mode == "selected_text" and not self.selected_text:
            raise ValueError("selected_text is required when mode is 'selected_text'")
        return self


# =============================================================================
# Response Models
# =============================================================================


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
        description="Relevant text snippet (truncated to 500 chars)",
    )

    relevance_score: float = Field(
        ge=0.0,
        le=1.0,
        description="Similarity score",
    )


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


# =============================================================================
# Streaming Event Models
# =============================================================================


class StreamChunk(BaseModel):
    """Streaming text chunk event."""

    type: Literal["chunk"] = "chunk"
    content: str = Field(description="Partial answer text")


class StreamSources(BaseModel):
    """Streaming sources event (sent at end)."""

    type: Literal["sources"] = "sources"
    sources: list[SourceCitation] = Field(description="Array of citations")


class StreamDone(BaseModel):
    """Streaming completion event."""

    type: Literal["done"] = "done"
    metadata: ResponseMetadata = Field(description="Response metadata")


class StreamError(BaseModel):
    """Streaming error event."""

    type: Literal["error"] = "error"
    error_code: str = Field(description="Machine-readable error code")
    message: str = Field(description="Human-readable error message")


# =============================================================================
# Error Response Models
# =============================================================================


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

    retry_after: int | None = Field(
        default=None,
        description="Seconds to wait before retry (for rate limiting)",
    )


# =============================================================================
# Health Check Models
# =============================================================================


class ServiceStatus(BaseModel):
    """Status of a single service."""

    status: Literal["up", "down", "degraded"] = Field(
        description="Service status",
    )

    latency_ms: float | None = Field(
        default=None,
        description="Service response latency in milliseconds",
    )

    message: str | None = Field(
        default=None,
        description="Status message or error details",
    )


class HealthResponse(BaseModel):
    """Response for /health endpoint."""

    status: Literal["healthy", "degraded", "unhealthy"] = Field(
        description="Overall system status",
    )

    services: dict[str, ServiceStatus] = Field(
        description="Status of each dependent service",
    )

    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Health check timestamp",
    )


# =============================================================================
# Conversation History Models (Spec-4)
# =============================================================================


class ConversationMessage(BaseModel):
    """A single message in the conversation history."""

    id: str = Field(description="Message UUID")

    role: Literal["user", "assistant"] = Field(description="Message sender")

    content: str = Field(description="Message content")

    sources: list[SourceCitation] | None = Field(
        default=None,
        description="Citations (for assistant messages only)",
    )

    mode: Literal["general", "selected_text"] = Field(description="Query mode")

    created_at: datetime = Field(description="Timestamp")


class ConversationsResponse(BaseModel):
    """Response for GET /conversations/{session_id}."""

    session_id: str = Field(description="Session identifier")

    messages: list[ConversationMessage] = Field(
        description="Conversation messages ordered oldest to newest",
    )
