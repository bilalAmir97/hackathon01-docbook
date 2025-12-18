# Quickstart: RAG Agent API Implementation

**Date**: 2025-12-16 | **Branch**: `006-rag-agent-api`

## Prerequisites

1. Spec-2 backend modules working (`backend/retrieval.py`, etc.)
2. Environment variables configured:
   - `GEMINI_API_KEY`
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `DATABASE_URL` (Neon Postgres)

## Step 1: Install Dependencies

Add to `backend/pyproject.toml`:

```toml
[project]
dependencies = [
    # Existing from Spec-2
    "cohere",
    "qdrant-client",
    "python-dotenv",

    # New for Spec-3
    "openai-agents",
    "fastapi>=0.109",
    "uvicorn[standard]>=0.27",
    "sse-starlette>=1.8",
    "asyncpg>=0.29",
    "pydantic>=2.0",
    "structlog>=24.1",
    "httpx>=0.27",  # For testing
]

[project.optional-dependencies]
test = [
    "pytest>=8.0",
    "pytest-asyncio>=0.23",
]
```

Install:
```bash
cd backend
uv pip install -e ".[test]"
```

## Step 2: Create agent.py

Create `backend/agent.py`:

```python
"""RAG Agent using OpenAI Agents SDK with Gemini backend."""

from __future__ import annotations

import os
from typing import Annotated

from agents import Agent, function_tool, Runner, set_tracing_disabled
from agents.extensions.models.litellm_model import LitellmModel
from dotenv import load_dotenv

# Import from Spec-2
from retrieval import retrieve
from models import RetrievalRequest

load_dotenv()

# Disable OpenAI tracing (using Gemini)
set_tracing_disabled(disabled=True)

# System prompts
GENERAL_SYSTEM_PROMPT = """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.

CRITICAL RULES:
1. ONLY use information from the search_documentation tool results
2. NEVER make claims not supported by retrieved content
3. ALWAYS cite sources for factual claims using [Source: page_title]
4. If information is not found, say "I couldn't find information about that in the documentation"

FORMAT:
- Answer clearly and concisely
- Include inline citations
"""

SELECTED_TEXT_SYSTEM_PROMPT = """You are analyzing a specific text selection provided by the user.

CRITICAL RULES:
1. ONLY answer based on the provided selected_text below
2. Do NOT use any external knowledge
3. If the answer is NOT in the selection, respond:
   "The provided selection does not contain information about [topic]."

Selected text:
---
{selected_text}
---
"""


@function_tool
def search_documentation(
    query: Annotated[str, "Search query for relevant documentation"],
    top_k: Annotated[int, "Number of results to retrieve (1-20)"] = 5,
) -> list[dict]:
    """Search the Physical AI & Humanoid Robotics documentation.

    Returns relevant text chunks with source metadata for citation.
    """
    request = RetrievalRequest(query=query, top_k=min(top_k, 20))
    result = retrieve(request)

    return [
        {
            "source_url": chunk.source_url,
            "page_title": chunk.page_title,
            "section_heading": chunk.section_heading or "Introduction",
            "chunk_text": chunk.chunk_text[:2000],
            "relevance_score": chunk.score,
        }
        for chunk in result.results
    ]


def get_model():
    """Get configured Gemini model via LiteLLM."""
    return LitellmModel(
        model=os.environ.get("GEMINI_MODEL", "gemini/gemini-2.0-flash"),
        api_key=os.environ["GEMINI_API_KEY"],
    )


async def run_agent(
    query: str,
    mode: str,
    selected_text: str | None = None,
    top_k: int = 5,
) -> tuple[str, list[dict]]:
    """Run the agent and return (answer, sources)."""

    if mode == "selected_text":
        agent = Agent(
            name="Selection Assistant",
            instructions=SELECTED_TEXT_SYSTEM_PROMPT.format(
                selected_text=selected_text
            ),
            tools=[],  # No tools
            model=get_model(),
        )
        sources = [{
            "source_url": "selected_text",
            "page_title": "User Selection",
            "section_heading": "Selected Text",
            "chunk_text": selected_text[:500] if selected_text else "",
            "relevance_score": 1.0,
        }]
    else:
        agent = Agent(
            name="RAG Assistant",
            instructions=GENERAL_SYSTEM_PROMPT,
            tools=[search_documentation],
            model=get_model(),
        )
        sources = []

    result = await Runner.run(agent, input=query)

    # Extract sources from tool calls
    if mode == "general":
        for item in result.run_items:
            if hasattr(item, 'output') and isinstance(item.output, list):
                sources.extend(item.output)

    return result.final_output, sources
```

## Step 3: Create api_models.py

Create `backend/api_models.py`:

```python
"""Pydantic models for API request/response."""

from datetime import datetime
from typing import Literal
from pydantic import BaseModel, Field, model_validator


class ChatRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=2000)
    session_id: str | None = None
    mode: Literal["general", "selected_text"] = "general"
    selected_text: str | None = Field(None, max_length=10000)
    top_k: int = Field(5, ge=1, le=20)
    score_threshold: float | None = Field(None, ge=0.0, le=1.0)
    filters: dict | None = None

    @model_validator(mode='after')
    def validate_mode(self):
        if self.mode == "selected_text" and not self.selected_text:
            raise ValueError("selected_text required for selected_text mode")
        return self


class SourceCitation(BaseModel):
    source_url: str
    page_title: str
    section_heading: str
    chunk_text: str = Field(max_length=500)
    relevance_score: float = Field(ge=0.0, le=1.0)


class ResponseMetadata(BaseModel):
    query_time_ms: float
    chunks_retrieved: int
    model: str


class ChatResponse(BaseModel):
    answer: str
    sources: list[SourceCitation]
    mode: str
    metadata: ResponseMetadata


class ErrorResponse(BaseModel):
    error_code: str
    message: str
    details: dict | None = None
    trace_id: str


class ServiceStatus(BaseModel):
    status: Literal["up", "down", "degraded"]
    latency_ms: float | None = None
    message: str | None = None


class HealthResponse(BaseModel):
    status: Literal["healthy", "degraded", "unhealthy"]
    services: dict[str, ServiceStatus]
    timestamp: datetime
```

## Step 4: Create api.py

Create `backend/api.py`:

```python
"""FastAPI endpoints for RAG Agent API."""

from __future__ import annotations

import time
import uuid
from datetime import datetime

from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse

from agent import run_agent
from api_models import (
    ChatRequest, ChatResponse, ErrorResponse,
    HealthResponse, ResponseMetadata, ServiceStatus, SourceCitation,
)

app = FastAPI(
    title="RAG Agent API",
    description="Question answering over Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
)


@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """Process a chat request and return an answer with citations."""
    trace_id = str(uuid.uuid4())[:8]
    start_time = time.perf_counter()

    try:
        answer, sources = await run_agent(
            query=request.query,
            mode=request.mode,
            selected_text=request.selected_text,
            top_k=request.top_k,
        )

        elapsed_ms = (time.perf_counter() - start_time) * 1000

        return ChatResponse(
            answer=answer,
            sources=[
                SourceCitation(
                    source_url=s["source_url"],
                    page_title=s["page_title"],
                    section_heading=s["section_heading"],
                    chunk_text=s["chunk_text"][:500],
                    relevance_score=s["relevance_score"],
                )
                for s in sources
            ],
            mode=request.mode,
            metadata=ResponseMetadata(
                query_time_ms=elapsed_ms,
                chunks_retrieved=len(sources),
                model="gemini-2.0-flash",
            ),
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                error_code="internal_error",
                message=str(e),
                trace_id=trace_id,
            ).model_dump(),
        )


@app.get("/health", response_model=HealthResponse)
async def health():
    """Check health of all dependent services."""
    services = {}
    overall = "healthy"

    # Check Qdrant
    try:
        from qdrant_client import QdrantClient
        import os
        client = QdrantClient(
            url=os.environ.get("QDRANT_URL"),
            api_key=os.environ.get("QDRANT_API_KEY"),
        )
        start = time.perf_counter()
        client.get_collections()
        latency = (time.perf_counter() - start) * 1000
        services["qdrant"] = ServiceStatus(status="up", latency_ms=latency)
    except Exception as e:
        services["qdrant"] = ServiceStatus(status="down", message=str(e))
        overall = "unhealthy"

    # Check Cohere
    try:
        import cohere
        import os
        client = cohere.Client(os.environ.get("COHERE_API_KEY"))
        start = time.perf_counter()
        client.embed(texts=["test"], model="embed-english-v3.0", input_type="search_query")
        latency = (time.perf_counter() - start) * 1000
        services["cohere"] = ServiceStatus(status="up", latency_ms=latency)
    except Exception as e:
        services["cohere"] = ServiceStatus(status="down", message=str(e))
        overall = "degraded" if overall == "healthy" else overall

    return HealthResponse(
        status=overall,
        services=services,
        timestamp=datetime.utcnow(),
    )
```

## Step 5: Update main.py

Update `backend/main.py`:

```python
"""FastAPI application entry point."""

import uvicorn
from api import app

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

## Step 6: Run the Server

```bash
cd backend
python main.py
```

Or with uvicorn directly:
```bash
uvicorn api:app --reload --port 8000
```

## Step 7: Test the API

```bash
# General mode
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "mode": "general"}'

# Selected-text mode
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is mentioned?", "mode": "selected_text", "selected_text": "ROS 2 is a robotics middleware."}'

# Health check
curl http://localhost:8000/health
```

## Step 8: View OpenAPI Documentation

Open in browser:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## Environment Variables (.env)

Create `backend/.env`:

```env
# Required
GEMINI_API_KEY=your-gemini-api-key
COHERE_API_KEY=your-cohere-api-key
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/dbname

# Optional
GEMINI_MODEL=gemini/gemini-2.0-flash
LOG_LEVEL=INFO
```

## Next Steps

1. Add streaming endpoint (`/chat/stream`)
2. Add database storage for conversations
3. Add structured logging
4. Write integration tests
5. Implement rate limiting
