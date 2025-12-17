# ADR-0006: Spec-2/Spec-3 Integration Pattern

- **Status:** Accepted
- **Date:** 2025-12-16
- **Feature:** 006-rag-agent-api
- **Context:** Spec-3 (RAG Agent API) needs to consume the retrieval functionality implemented in Spec-2. Three integration patterns were considered: library import, HTTP/REST calls, or reimplementation. The decision affects latency, coupling, maintainability, and deployment architecture.

## Decision

**Use direct library import** for Spec-2/Spec-3 integration.

Spec-3 will import Spec-2's retrieval module as a Python library rather than calling it via HTTP or reimplementing the retrieval logic.

### Integration Contract

```python
from backend.retrieval import (
    RetrievalRequest,      # Query parameters dataclass
    SearchResult,          # Results container with chunks
    ChunkResult,           # Individual chunk with metadata
    retrieve_chunks,       # Main retrieval function
)
```

### Type Mapping

| Spec-2 Type | Spec-3 Usage |
|-------------|--------------|
| `RetrievalRequest` | Used internally by `search_documentation` agent tool |
| `ChunkResult` | Mapped to `SourceCitation` in API responses |
| `SearchResult` | Provides results list and timing metadata |

## Alternatives Considered

### Option A: Library Import (Selected ✅)

Import Spec-2 as a Python module within the same process.

**Pros:**
- Zero network latency between retrieval and agent
- Shared type definitions (no serialization/deserialization)
- Single deployment unit simplifies operations
- Direct error propagation with stack traces

**Cons:**
- Tighter coupling between Spec-2 and Spec-3
- Cannot scale retrieval independently
- Changes to Spec-2 require Spec-3 redeployment

### Option B: HTTP/REST Calls (Rejected ❌)

Deploy Spec-2 as a separate service with HTTP endpoints.

**Pros:**
- Loose coupling, independent deployment
- Can scale retrieval service horizontally
- Language-agnostic (could rewrite in different language)

**Cons:**
- Network latency overhead (~50-200ms per call)
- Additional error handling for network failures
- Serialization/deserialization overhead
- More complex deployment (two services)
- Need to maintain API versioning

### Option C: Reimplementation (Rejected ❌)

Copy retrieval logic into Spec-3 codebase.

**Pros:**
- Full control over implementation
- No external dependencies

**Cons:**
- Code duplication violates DRY principle
- Drift risk: Spec-2 and Spec-3 implementations diverge
- Bug fixes needed in multiple places
- Larger maintenance burden

## Consequences

### Positive

- **Performance**: No network overhead; retrieval calls are in-process function calls
- **Simplicity**: Single deployment artifact (FastAPI app with retrieval module)
- **Type Safety**: Shared Python types between retrieval and agent layers
- **Debugging**: Full stack traces when errors occur in retrieval
- **Consistency**: Single source of truth for retrieval logic

### Negative

- **Coupling**: Spec-3 depends directly on Spec-2 module structure
- **Scaling**: Cannot scale retrieval independently of API
- **Deployment**: Any Spec-2 change requires Spec-3 redeployment

### Mitigations

1. **Stable Interface**: Define clear interface contract (`retrieve_chunks` function signature) that Spec-2 must maintain
2. **Versioning**: If breaking changes needed, version the retrieval module interface
3. **Future Path**: If scaling becomes necessary, can extract to HTTP service later (interface stays similar)

## Module Structure

```
backend/
├── retrieval/           # Spec-2 module
│   ├── __init__.py      # Exports public interface
│   ├── client.py        # Qdrant/Cohere clients
│   ├── models.py        # RetrievalRequest, SearchResult, ChunkResult
│   └── search.py        # retrieve_chunks function
├── agent/               # Spec-3 module
│   ├── __init__.py
│   ├── api.py           # FastAPI endpoints
│   ├── agent.py         # OpenAI Agents SDK setup
│   └── tools.py         # search_documentation tool (uses retrieval)
└── main.py              # FastAPI application entry point
```

## Related Decisions

- ADR-0005: Idempotency Strategy (Spec-1 → Spec-2 data contract)
- Spec-2 spec.md: Interface contracts for RetrievalRequest/SearchResult
- Spec-3 spec.md: Spec-2 Integration Pattern section
