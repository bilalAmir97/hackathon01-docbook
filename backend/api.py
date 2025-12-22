"""FastAPI endpoints for RAG Agent API.

This module provides the HTTP API endpoints for the RAG Agent:
- POST /chat - Synchronous chat with citations
- POST /chat/stream - Server-Sent Events streaming
- GET /health - Service health check
"""

from __future__ import annotations

# CRITICAL: Apply Pydantic patch BEFORE any OpenAI SDK imports
import pydantic_patch  # noqa: F401

import json
import os
import time
import uuid
from contextlib import asynccontextmanager
from datetime import datetime
from typing import AsyncGenerator

import structlog
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse

from agent import run_agent, run_agent_streamed, validate_citations
from api_models import (
    ChatRequest,
    ChatResponse,
    ConversationMessage,
    ConversationsResponse,
    ErrorResponse,
    HealthResponse,
    ResponseMetadata,
    ServiceStatus,
    SourceCitation,
    StreamChunk,
    StreamDone,
    StreamError,
    StreamSources,
)
from database import get_conversation_history, init_pool, close_pool
from errors import (
    EmbeddingError,
    QdrantConnectionError,
    QdrantTimeoutError,
    RetrievalError,
)

# Load environment variables
load_dotenv()

# Configure structured logging
structlog.configure(
    processors=[
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.JSONRenderer(),
    ],
    wrapper_class=structlog.stdlib.BoundLogger,
    context_class=dict,
    logger_factory=structlog.stdlib.LoggerFactory(),
    cache_logger_on_first_use=True,
)

logger = structlog.get_logger()

# =============================================================================
# FastAPI Application with Lifespan
# =============================================================================


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application startup and shutdown events."""
    # Startup: Initialize database connection pool
    logger.info("Starting up RAG Agent API...")
    await init_pool()
    yield
    # Shutdown: Close database connection pool
    logger.info("Shutting down RAG Agent API...")
    await close_pool()


app = FastAPI(
    title="RAG Agent API",
    description="Question answering over Physical AI & Humanoid Robotics textbook with citations",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    lifespan=lifespan,
)

# CORS middleware for frontend access - environment-aware origin allowlist
# Dev origins: localhost:3000, 127.0.0.1:3000, localhost:3001 (Docusaurus dev server)
# Prod origins: Load from ALLOWED_ORIGINS environment variable (comma-separated)
ALLOWED_ORIGINS = os.getenv(
    "ALLOWED_ORIGINS",
    "http://localhost:3000,http://127.0.0.1:3000,http://localhost:3001"
).split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=[origin.strip() for origin in ALLOWED_ORIGINS if origin.strip()],
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
    max_age=600,  # Cache preflight requests for 10 minutes
)


# =============================================================================
# Helper Functions
# =============================================================================


def generate_trace_id() -> str:
    """Generate a short trace ID for request tracking."""
    return str(uuid.uuid4())[:8]


def format_citations(sources: list[dict]) -> list[SourceCitation]:
    """Convert raw source dicts to SourceCitation models.

    Args:
        sources: List of source dictionaries from agent

    Returns:
        List of validated SourceCitation models
    """
    citations = []
    for source in sources:
        try:
            # Truncate chunk_text to 500 chars
            chunk_text = source.get("chunk_text", "")[:500]

            # Clamp relevance_score to valid range
            relevance_score = max(0.0, min(1.0, source.get("relevance_score", 0.0)))

            citation = SourceCitation(
                source_url=source.get("source_url", ""),
                page_title=source.get("page_title", ""),
                section_heading=source.get("section_heading", ""),
                chunk_text=chunk_text,
                relevance_score=relevance_score,
            )
            citations.append(citation)
        except Exception:
            # Skip malformed sources
            continue

    return citations


def format_sse_event(data: dict) -> str:
    """Format data as Server-Sent Events message.

    Args:
        data: Dictionary to send as JSON

    Returns:
        SSE-formatted string
    """
    return f"data: {json.dumps(data)}\n\n"


def hash_query(query: str) -> str:
    """Create a privacy-preserving hash of the query for logging."""
    import hashlib

    return hashlib.sha256(query.encode()).hexdigest()[:8]


# =============================================================================
# Exception Handlers
# =============================================================================


@app.exception_handler(QdrantConnectionError)
async def qdrant_connection_error_handler(
    request: Request, exc: QdrantConnectionError
) -> JSONResponse:
    """Handle Qdrant connection errors with 503."""
    trace_id = getattr(request.state, "trace_id", generate_trace_id())
    logger.error("qdrant_connection_error", trace_id=trace_id, error=str(exc))

    return JSONResponse(
        status_code=503,
        content=ErrorResponse(
            error_code="retrieval_unavailable",
            message="Qdrant service is temporarily unavailable",
            details={"retry_after": 30},
            trace_id=trace_id,
        ).model_dump(),
    )


@app.exception_handler(QdrantTimeoutError)
async def qdrant_timeout_error_handler(
    request: Request, exc: QdrantTimeoutError
) -> JSONResponse:
    """Handle Qdrant timeout errors with 503."""
    trace_id = getattr(request.state, "trace_id", generate_trace_id())
    logger.error("qdrant_timeout_error", trace_id=trace_id, error=str(exc))

    return JSONResponse(
        status_code=503,
        content=ErrorResponse(
            error_code="retrieval_unavailable",
            message="Qdrant search timed out",
            details={"retry_after": 10},
            trace_id=trace_id,
        ).model_dump(),
    )


@app.exception_handler(EmbeddingError)
async def embedding_error_handler(
    request: Request, exc: EmbeddingError
) -> JSONResponse:
    """Handle embedding errors with 503."""
    trace_id = getattr(request.state, "trace_id", generate_trace_id())
    logger.error("embedding_error", trace_id=trace_id, error=str(exc))

    return JSONResponse(
        status_code=503,
        content=ErrorResponse(
            error_code="embedding_failed",
            message="Cohere embedding service is temporarily unavailable",
            details={"retry_after": 10},
            trace_id=trace_id,
        ).model_dump(),
    )


@app.exception_handler(RetrievalError)
async def retrieval_error_handler(
    request: Request, exc: RetrievalError
) -> JSONResponse:
    """Handle generic retrieval errors."""
    trace_id = getattr(request.state, "trace_id", generate_trace_id())
    logger.error("retrieval_error", trace_id=trace_id, code=exc.code, error=str(exc))

    return JSONResponse(
        status_code=503 if exc.is_retryable else 500,
        content=ErrorResponse(
            error_code=exc.code,
            message=exc.message,
            details=exc.details,
            trace_id=trace_id,
        ).model_dump(),
    )


# =============================================================================
# Endpoints
# =============================================================================


@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    """Process a chat request and return an answer with citations.

    Supports two modes:
    - general: Uses Qdrant retrieval to find relevant content
    - selected_text: Uses only the provided text selection
    """
    trace_id = generate_trace_id()
    start_time = time.perf_counter()

    logger.info(
        "chat_request",
        trace_id=trace_id,
        query_hash=hash_query(request.query),
        mode=request.mode,
        top_k=request.top_k,
    )

    try:
        # Run the agent
        answer, sources = await run_agent(
            query=request.query,
            mode=request.mode,
            selected_text=request.selected_text,
            top_k=request.top_k,
            score_threshold=request.score_threshold,
            filters=request.filters,
        )

        # The sources variable already contains the chunks retrieved by the agent's search_documentation tool
        # No additional validation needed since these are the actual retrieved chunks

        # Calculate elapsed time
        elapsed_ms = (time.perf_counter() - start_time) * 1000

        # Format response
        response = ChatResponse(
            answer=answer,
            sources=format_citations(sources),
            mode=request.mode,
            metadata=ResponseMetadata(
                query_time_ms=round(elapsed_ms, 2),
                chunks_retrieved=len(sources),
                model=os.environ.get("GEMINI_MODEL", "gemini-2.0-flash"),
            ),
        )

        logger.info(
            "chat_response",
            trace_id=trace_id,
            latency_ms=round(elapsed_ms, 2),
            chunks_retrieved=len(sources),
            sources_cited=len(response.sources),
        )

        return response

    except ValueError as e:
        # Validation errors
        logger.warning("chat_validation_error", trace_id=trace_id, error=str(e))
        raise HTTPException(
            status_code=400,
            detail=ErrorResponse(
                error_code="validation_error",
                message=str(e),
                trace_id=trace_id,
            ).model_dump(),
        )

    except Exception as e:
        # Unexpected errors
        elapsed_ms = (time.perf_counter() - start_time) * 1000
        logger.error(
            "chat_error",
            trace_id=trace_id,
            error=str(e),
            latency_ms=round(elapsed_ms, 2),
        )
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                error_code="internal_error",
                message="An unexpected error occurred",
                details={"error": str(e)},
                trace_id=trace_id,
            ).model_dump(),
        )


@app.post("/chat/stream")
async def chat_stream(request: ChatRequest) -> StreamingResponse:
    """Stream a chat response with Server-Sent Events.

    Returns progressive text chunks followed by sources and completion metadata.

    Event types:
    - chunk: Partial answer text
    - sources: Array of citations (sent at end)
    - done: Completion with metadata
    - error: Error occurred
    """
    trace_id = generate_trace_id()
    start_time = time.perf_counter()

    logger.info(
        "chat_stream_request",
        trace_id=trace_id,
        query_hash=hash_query(request.query),
        mode=request.mode,
    )

    async def event_generator() -> AsyncGenerator[str, None]:
        try:
            sources = []

            async for event in run_agent_streamed(
                query=request.query,
                mode=request.mode,
                selected_text=request.selected_text,
                top_k=request.top_k,
                score_threshold=request.score_threshold,
                filters=request.filters,
            ):
                if event["type"] == "chunk":
                    yield format_sse_event(
                        StreamChunk(content=event["content"]).model_dump()
                    )
                elif event["type"] == "sources":
                    sources = event["sources"]
                    yield format_sse_event(
                        StreamSources(
                            sources=format_citations(sources)
                        ).model_dump()
                    )

            # Send completion event
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            yield format_sse_event(
                StreamDone(
                    metadata=ResponseMetadata(
                        query_time_ms=round(elapsed_ms, 2),
                        chunks_retrieved=len(sources),
                        model=os.environ.get("GEMINI_MODEL", "gemini-2.0-flash"),
                    )
                ).model_dump()
            )

            logger.info(
                "chat_stream_complete",
                trace_id=trace_id,
                latency_ms=round(elapsed_ms, 2),
            )

        except Exception as e:
            logger.error("chat_stream_error", trace_id=trace_id, error=str(e))
            yield format_sse_event(
                StreamError(
                    error_code="stream_error",
                    message=str(e),
                ).model_dump()
            )

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Trace-Id": trace_id,
        },
    )


@app.get("/health", response_model=HealthResponse)
async def health() -> HealthResponse:
    """Check health of all dependent services.

    Returns status for:
    - qdrant: Vector database connectivity
    - cohere: Embedding service connectivity
    """
    services: dict[str, ServiceStatus] = {}
    overall = "healthy"

    # Check Qdrant
    try:
        from qdrant_client import QdrantClient

        qdrant_url = os.environ.get("QDRANT_URL")
        qdrant_api_key = os.environ.get("QDRANT_API_KEY")

        if not qdrant_url or not qdrant_api_key:
            services["qdrant"] = ServiceStatus(
                status="down",
                message="QDRANT_URL or QDRANT_API_KEY not configured",
            )
            overall = "unhealthy"
        else:
            client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
            start = time.perf_counter()
            client.get_collections()
            latency = (time.perf_counter() - start) * 1000
            services["qdrant"] = ServiceStatus(status="up", latency_ms=round(latency, 2))

    except Exception as e:
        services["qdrant"] = ServiceStatus(status="down", message=str(e))
        overall = "unhealthy"

    # Check Cohere
    try:
        import cohere

        cohere_api_key = os.environ.get("COHERE_API_KEY")

        if not cohere_api_key:
            services["cohere"] = ServiceStatus(
                status="down",
                message="COHERE_API_KEY not configured",
            )
            overall = "degraded" if overall == "healthy" else overall
        else:
            client = cohere.Client(cohere_api_key)
            start = time.perf_counter()
            client.embed(
                texts=["health check"],
                model="embed-english-v3.0",
                input_type="search_query",
            )
            latency = (time.perf_counter() - start) * 1000
            services["cohere"] = ServiceStatus(status="up", latency_ms=round(latency, 2))

    except Exception as e:
        services["cohere"] = ServiceStatus(status="down", message=str(e))
        overall = "degraded" if overall == "healthy" else overall

    # Check Gemini API key (basic check - no API call)
    gemini_key = os.environ.get("GEMINI_API_KEY")
    if gemini_key:
        services["gemini"] = ServiceStatus(status="up", message="API key configured")
    else:
        services["gemini"] = ServiceStatus(
            status="down",
            message="GEMINI_API_KEY not configured",
        )
        overall = "degraded" if overall == "healthy" else overall

    return HealthResponse(
        status=overall,
        services=services,
        timestamp=datetime.utcnow(),
    )


# =============================================================================
# Conversation History Endpoint (Spec-4)
# =============================================================================


@app.get("/conversations/{session_id}", response_model=ConversationsResponse)
async def get_conversations(
    session_id: str,
    limit: int = 20,
) -> ConversationsResponse:
    """Retrieve conversation history for a session.

    Returns messages ordered oldest to newest for display.

    Args:
        session_id: UUID v4 identifier for the conversation session
        limit: Maximum number of conversation turns to retrieve (max 20)

    Returns:
        ConversationsResponse with session_id and messages array

    Raises:
        HTTPException 400: Invalid session_id format
    """
    trace_id = generate_trace_id()

    # Validate UUID v4 format
    try:
        uuid.UUID(session_id, version=4)
    except ValueError:
        logger.warning(
            "invalid_session_id",
            trace_id=trace_id,
            session_id=session_id,
        )
        raise HTTPException(
            status_code=400,
            detail=ErrorResponse(
                error_code="invalid_session_id",
                message="Invalid UUID format. Expected UUID v4.",
                trace_id=trace_id,
            ).model_dump(),
        )

    # Enforce maximum limit
    limit = min(limit, 20)

    # Get history from database
    history = await get_conversation_history(session_id, limit=limit)

    # Transform to API format with user/assistant message pairs
    messages: list[ConversationMessage] = []
    for conv in history:
        # Parse created_at timestamp
        created_at = datetime.utcnow()
        if conv.get("created_at"):
            try:
                created_at = datetime.fromisoformat(conv["created_at"])
            except (ValueError, TypeError):
                pass

        # User message
        messages.append(
            ConversationMessage(
                id=str(uuid.uuid4()),
                role="user",
                content=conv["query"],
                sources=None,
                mode=conv["mode"],
                created_at=created_at,
            )
        )

        # Assistant message
        sources = None
        if conv.get("sources"):
            try:
                sources = [
                    SourceCitation(**s) for s in conv["sources"]
                ] if conv["sources"] else None
            except (ValueError, TypeError):
                # Skip malformed sources
                sources = None

        messages.append(
            ConversationMessage(
                id=str(uuid.uuid4()),
                role="assistant",
                content=conv["response"],
                sources=sources,
                mode=conv["mode"],
                created_at=created_at,
            )
        )

    logger.info(
        "conversation_history_retrieved",
        trace_id=trace_id,
        session_id=session_id,
        message_count=len(messages),
    )

    return ConversationsResponse(session_id=session_id, messages=messages)


# =============================================================================
# Root Endpoint
# =============================================================================


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "RAG Agent API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health",
    }
