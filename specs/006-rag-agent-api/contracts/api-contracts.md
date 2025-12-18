# API Contracts: RAG Agent API

**Version**: 1.0.0 | **Date**: 2025-12-16

## Base URL

```
http://localhost:8000
```

---

## Endpoints

### POST /chat

Process a chat request and return an answer with citations.

#### Request

```http
POST /chat HTTP/1.1
Content-Type: application/json

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

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| query | string | Yes | User's question (1-2000 chars) |
| session_id | string | No | UUID for conversation grouping |
| mode | string | No | "general" (default) or "selected_text" |
| selected_text | string | Conditional | Required if mode="selected_text" |
| top_k | integer | No | Retrieval count (1-20, default 5) |
| score_threshold | float | No | Min similarity (0.0-1.0) |
| filters | object | No | Metadata filters |

#### Response (200 OK)

```json
{
  "answer": "ROS 2 (Robot Operating System 2) is an open-source robotics middleware framework...",
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

#### Error Responses

**400 Bad Request - Validation Error**
```json
{
  "error_code": "validation_error",
  "message": "Query must be at least 1 character",
  "details": {"field": "query"},
  "trace_id": "abc123"
}
```

**503 Service Unavailable**
```json
{
  "error_code": "retrieval_unavailable",
  "message": "Qdrant service is temporarily unavailable",
  "details": {"retry_after": 30},
  "trace_id": "abc123"
}
```

---

### POST /chat/stream

Stream a chat response with Server-Sent Events.

#### Request

Same as `/chat`

#### Response (200 OK, text/event-stream)

```
data: {"type": "chunk", "content": "ROS 2 is "}

data: {"type": "chunk", "content": "a robotics middleware..."}

data: {"type": "sources", "sources": [{"source_url": "...", ...}]}

data: {"type": "done", "metadata": {"query_time_ms": 1250.5, ...}}
```

Event Types:
- `chunk`: Partial answer text
- `sources`: Array of citations (sent at end)
- `done`: Completion with metadata
- `error`: Error occurred

---

### GET /health

Check health of all dependent services.

#### Response (200 OK)

```json
{
  "status": "healthy",
  "services": {
    "qdrant": {
      "status": "up",
      "latency_ms": 45.2
    },
    "cohere": {
      "status": "up",
      "latency_ms": 120.5
    },
    "gemini": {
      "status": "up",
      "latency_ms": 200.1
    },
    "database": {
      "status": "up",
      "latency_ms": 30.0
    }
  },
  "timestamp": "2025-12-16T10:30:00Z"
}
```

Status Values:
- `healthy`: All services operational
- `degraded`: Some services impaired
- `unhealthy`: Critical services down

---

## Error Taxonomy

| Code | HTTP Status | Condition | Retryable |
|------|-------------|-----------|-----------|
| validation_error | 400 | Invalid request | No |
| retrieval_unavailable | 503 | Qdrant down | Yes |
| embedding_failed | 503 | Cohere down | Yes |
| agent_unavailable | 503 | Gemini down | Yes |
| rate_limited | 429 | Too many requests | Yes |
| internal_error | 500 | Unexpected error | No |

---

## OpenAPI Specification

```yaml
openapi: 3.0.3
info:
  title: RAG Agent API
  version: 1.0.0
  description: Question answering over Physical AI & Humanoid Robotics textbook

paths:
  /chat:
    post:
      summary: Process chat request
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ChatRequest'
      responses:
        '200':
          description: Successful response
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ChatResponse'
        '400':
          description: Validation error
        '503':
          description: Service unavailable

  /chat/stream:
    post:
      summary: Stream chat response
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ChatRequest'
      responses:
        '200':
          description: SSE stream
          content:
            text/event-stream:
              schema:
                type: string

  /health:
    get:
      summary: Health check
      responses:
        '200':
          description: Health status
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/HealthResponse'

components:
  schemas:
    ChatRequest:
      type: object
      required: [query]
      properties:
        query:
          type: string
          minLength: 1
          maxLength: 2000
        session_id:
          type: string
          format: uuid
        mode:
          type: string
          enum: [general, selected_text]
          default: general
        selected_text:
          type: string
          maxLength: 10000
        top_k:
          type: integer
          minimum: 1
          maximum: 20
          default: 5

    ChatResponse:
      type: object
      properties:
        answer:
          type: string
        sources:
          type: array
          items:
            $ref: '#/components/schemas/SourceCitation'
        mode:
          type: string
        metadata:
          $ref: '#/components/schemas/ResponseMetadata'

    SourceCitation:
      type: object
      properties:
        source_url:
          type: string
        page_title:
          type: string
        section_heading:
          type: string
        chunk_text:
          type: string
        relevance_score:
          type: number

    ResponseMetadata:
      type: object
      properties:
        query_time_ms:
          type: number
        chunks_retrieved:
          type: integer
        model:
          type: string

    HealthResponse:
      type: object
      properties:
        status:
          type: string
          enum: [healthy, degraded, unhealthy]
        services:
          type: object
        timestamp:
          type: string
          format: date-time

    ErrorResponse:
      type: object
      properties:
        error_code:
          type: string
        message:
          type: string
        details:
          type: object
        trace_id:
          type: string
```

---

## Example Requests

### cURL Examples

**General Mode:**
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "mode": "general",
    "top_k": 5
  }'
```

**Selected-Text Mode:**
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What framework is described?",
    "mode": "selected_text",
    "selected_text": "ROS 2 is a robotics middleware that provides tools and libraries for robot software development."
  }'
```

**Streaming:**
```bash
curl -X POST http://localhost:8000/chat/stream \
  -H "Content-Type: application/json" \
  -H "Accept: text/event-stream" \
  -d '{
    "query": "Explain Gazebo simulation",
    "mode": "general"
  }'
```

**Health Check:**
```bash
curl http://localhost:8000/health
```

### Python Examples

```python
import httpx

# Non-streaming
async def ask_question(query: str):
    async with httpx.AsyncClient() as client:
        response = await client.post(
            "http://localhost:8000/chat",
            json={"query": query, "mode": "general"}
        )
        return response.json()

# Streaming
async def ask_streaming(query: str):
    async with httpx.AsyncClient() as client:
        async with client.stream(
            "POST",
            "http://localhost:8000/chat/stream",
            json={"query": query, "mode": "general"}
        ) as response:
            async for line in response.aiter_lines():
                if line.startswith("data: "):
                    data = json.loads(line[6:])
                    yield data
```
