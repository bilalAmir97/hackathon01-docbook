# Research: RAG Agent API

**Date**: 2025-12-16 | **Branch**: `006-rag-agent-api`

## Research Objectives

1. Understand OpenAI Agents SDK tool wiring patterns
2. Evaluate Gemini integration via OpenAI-compatible endpoint
3. Define citation extraction strategy
4. Design selected-text-only enforcement

---

## OpenAI Agents SDK Findings

### Source: Context7 MCP (Official Documentation)

**Key Pattern: @function_tool Decorator**

The OpenAI Agents SDK uses decorators to convert Python functions into agent tools:

```python
from agents import Agent, function_tool, Runner
from typing import Annotated

@function_tool
def get_weather(city: Annotated[str, "The city to get weather for"]) -> str:
    """Get current weather for a city."""
    return f"Weather in {city}: Sunny, 72F"

agent = Agent(
    name="Assistant",
    instructions="You are helpful.",
    tools=[get_weather],
)
```

**Key Findings:**
1. `@function_tool` decorator automatically generates JSON schema from type hints
2. `Annotated` provides parameter descriptions for LLM
3. Docstrings become tool descriptions
4. Tools can return primitive types, dicts, or Pydantic models

### Model Provider Integration

For non-OpenAI models (like Gemini), use LiteLLM:

```python
from agents.extensions.models.litellm_model import LitellmModel

agent = Agent(
    model=LitellmModel(
        model="gemini/gemini-2.0-flash",
        api_key=os.environ["GEMINI_API_KEY"],
    ),
)
```

### Streaming Pattern

```python
from openai.types.responses import ResponseTextDeltaEvent

result = Runner.run_streamed(agent, input="...")
async for event in result.stream_events():
    if event.type == "raw_response_event":
        if isinstance(event.data, ResponseTextDeltaEvent):
            print(event.data.delta, end="")
```

### Tracing Disabled

For Gemini backend, OpenAI tracing is not supported:

```python
from agents import set_tracing_disabled
set_tracing_disabled(disabled=True)
```

---

## Spec-2 Integration Analysis

### Existing Retrieval Interface

From `backend/retrieval.py`:

```python
def retrieve(
    request: RetrievalRequest,
    cohere_client: cohere.Client | None = None,
    qdrant_client: QdrantClient | None = None,
) -> SearchResult:
```

**Key Types (from models.py):**

- `RetrievalRequest`: query, top_k, score_threshold, filters
- `ChunkResult`: id, score, chunk_text, source_url, page_title, section_heading, chunk_index
- `SearchResult`: query, results[], total_found, latency_ms, embedding_ms, search_ms

**Integration Decision**: Direct library import (not HTTP calls)
- No network overhead
- Shared type definitions
- Same deployment unit

---

## Citation Extraction Strategy

### Challenge
Agent generates free-text with inline citations. Need to extract structured sources.

### Approaches Considered

1. **Regex Extraction**: Parse `[Source: Title]` patterns
   - Fragile, may miss variations

2. **Tool Output Capture**: Use tool call results as sources
   - More reliable, direct from retrieval
   - Selected approach

3. **Structured Output**: Force agent to return JSON
   - Complex, reduces response quality

### Selected Approach

Capture tool outputs during agent run:

```python
def extract_sources_from_result(result) -> list[dict]:
    sources = []
    for item in result.run_items:
        if hasattr(item, 'output') and isinstance(item.output, list):
            sources.extend(item.output)
    return sources
```

---

## Selected-Text-Only Enforcement

### Requirement
When `mode="selected_text"`, agent must ONLY use provided text, never external sources.

### Approaches Considered

1. **Runtime Filtering**: Let agent call tools, filter results
   - Risk: Agent may still use retrieved info

2. **Empty Results**: Return empty from tool
   - Confusing to agent

3. **Tool Removal**: Create agent without tools
   - Clean, deterministic
   - Selected approach

### Selected Approach

Create separate agent configuration:

```python
# No tools = no retrieval capability
selected_text_agent = Agent(
    tools=[],  # Empty
    instructions=SELECTED_TEXT_INSTRUCTIONS.format(selected_text=text),
)
```

System prompt explicitly constrains to selection:
- "ONLY answer based on the provided selected_text"
- "Do NOT use any external knowledge"
- Fallback: "Selection doesn't contain information about..."

---

## Error Handling Strategy

### External Service Failures

| Service | Error | Handling |
|---------|-------|----------|
| Qdrant | Connection timeout | Retry 3x, then 503 |
| Cohere | API error | Retry 2x, then 503 |
| Gemini | Rate limit | Backoff, retry |

### "No Hallucinated Citations" Rule

Post-processing validation:
1. Extract all source_url from response
2. Compare against retrieved chunks
3. Strip any URL not in retrieval results
4. Log warning for discrepancies

```python
def validate_citations(response_sources, retrieved_chunks):
    valid_urls = {c["source_url"] for c in retrieved_chunks}
    validated = []
    for source in response_sources:
        if source["source_url"] in valid_urls or source["source_url"] == "selected_text":
            validated.append(source)
        else:
            logger.warning("citation_stripped", url=source["source_url"])
    return validated
```

---

## Performance Considerations

### Latency Budget (3s total)

| Component | Target | Notes |
|-----------|--------|-------|
| Embedding | 500ms | Cohere API |
| Qdrant search | 200ms | Vector search |
| LLM generation | 2000ms | Gemini |
| Overhead | 300ms | Network, processing |

### Streaming Benefits
- First chunk in <1s
- Perceived latency improved
- Citations at end acceptable

---

## Conclusions

1. **OpenAI Agents SDK** is well-suited for this use case
2. **LiteLLM** provides clean Gemini integration
3. **Tool output capture** is reliable for citations
4. **Tool removal** is safest for selected-text mode
5. **Streaming** improves UX significantly
