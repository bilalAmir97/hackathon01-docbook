# Implementation Plan: Retrieval Pipeline + Validation

**Branch**: `005-spec02-retrieval-validation` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-spec02-retrieval-validation/spec.md`

**Note**: This template is filled in by the `/sp.plan` command.

## Summary

Build a retrieval interface over the existing Qdrant `rag_embedding` collection (populated by Spec-1) that performs semantic similarity search with metadata filtering, and a validation/evaluation harness to verify pipeline correctness. The retrieval module will be added to the existing `backend/` directory as `retrieval.py` with supporting modules for errors, validation, and evaluation. Query embeddings use Cohere `embed-english-v3.0` with `input_type="search_query"` (vs `search_document` used in ingestion).

## Technical Context

**Language/Version**: Python 3.11 (managed via `uv`, matches Spec-1)
**Primary Dependencies**: `cohere` (query embeddings), `qdrant-client` (vector search), `python-dotenv` (env vars), `pytest` (testing)
**Storage**: Qdrant Cloud (collection: `rag_embedding`, read-only access)
**Testing**: pytest with fixtures for mocked clients, golden test set for evaluation
**Target Platform**: CLI tool + Python module (cross-platform: Windows/Linux/macOS)
**Project Type**: Extension to existing backend (`backend/retrieval.py` + supporting modules)
**Performance Goals**: <2000ms p95 query latency, <500ms embedding, <200ms search
**Constraints**: Read-only Qdrant access, match Spec-1 embedding model, 5s total timeout
**Scale/Scope**: ~100-200 vectors in collection, 15+ golden test queries, 72 test cases

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Embodied Intelligence | N/A | Infrastructure for RAG retrieval, not robotics content |
| II. Simulation-First | N/A | Backend pipeline, no simulation involved |
| III. Technical Rigor | PASS | Python code will be syntactically correct and reproducible |
| IV. Clarity & Structure | PASS | Modular design with clear separation of concerns |
| V. Accessibility & Readability | PASS | Docstrings, type hints, error taxonomy |
| VI. Verification & Citation | PASS | Uses official Cohere/Qdrant APIs, validated against Spec-1 |

**Gate Result**: PASS - No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/005-spec02-retrieval-validation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── retrieval-api.md # Interface contracts
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── main.py                    # Spec-1 ingestion pipeline (existing)
├── retrieval.py               # NEW: Core retrieval interface
├── errors.py                  # NEW: Error taxonomy and exceptions
├── validation.py              # NEW: Health checks and validation suite
├── evaluation.py              # NEW: Evaluation harness and reporting
├── models.py                  # NEW: Dataclasses for contracts
├── config.py                  # NEW: Shared configuration
├── pyproject.toml             # Existing (add pytest dependency)
├── .env.example               # Existing (already has required vars)
└── .python-version            # Existing (3.11)

tests/
├── __init__.py
├── conftest.py                # NEW: pytest fixtures
├── test_retrieval.py          # NEW: Unit tests for retrieval
├── test_validation.py         # NEW: Unit tests for validation
├── test_errors.py             # NEW: Unit tests for error handling
├── test_evaluation.py         # NEW: Unit tests for evaluation
├── golden_queries.json        # NEW: Golden test set (15+ queries)
└── integration/
    └── test_e2e_retrieval.py  # NEW: End-to-end integration tests
```

**Structure Decision**: Modular extension to existing backend. Retrieval logic separated into dedicated modules while reusing Spec-1's environment and dependencies. Tests in separate `tests/` directory following pytest conventions.

## Architecture Overview

### Data Flow

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ RetrievalRequest│────>│    embed()      │────>│  qdrant.search  │
│  (query, k,     │     │ (Cohere query   │     │  (with filters) │
│   filters)      │     │  input_type)    │     │                 │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                                        │
                                                        v
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  SearchResult   │<────│  ChunkResult[]  │<────│   Qdrant Points │
│  (with timing)  │     │  (ranked)       │     │   (payloads)    │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

### Module Responsibilities

| Module | Responsibility |
|--------|----------------|
| `retrieval.py` | Core `retrieve()` function, filter building, result mapping |
| `errors.py` | Exception hierarchy, error codes E001-E010, retry logic |
| `validation.py` | Health checks (connectivity, collection, schema, retrieval) |
| `evaluation.py` | Golden test runner, MRR calculation, report generation |
| `models.py` | Dataclasses: RetrievalRequest, ChunkResult, SearchResult, etc. |
| `config.py` | Constants, timeouts, environment loading |

### Key Design Decisions

1. **Modular Architecture**: Separate files for retrieval, validation, and evaluation (unlike Spec-1's single-file)
2. **Query Embedding Input Type**: `input_type="search_query"` for queries (Spec-1 uses `search_document`)
3. **Deterministic Results**: Fixed ordering by score descending, then by point ID for ties
4. **Typed Exceptions**: Error taxonomy with 10 error codes for clear error handling
5. **Timing Breakdown**: Track embedding_ms and search_ms separately in SearchResult

### Error Handling Strategy

| Error Type | Handling | Retry |
|------------|----------|-------|
| QdrantConnectionError (E001) | Log, raise with guidance | 3x exponential backoff |
| CollectionNotFoundError (E002) | Raise immediately | No |
| InvalidQueryError (E003) | Validate before API call | No |
| EmbeddingError (E004) | Log, retry with backoff | 2x |
| InvalidFilterError (E005) | Validate field names | No |
| QdrantTimeoutError (E006) | Raise after timeout | 1x |
| RateLimitError (E008) | Backoff and retry | Yes, with wait |

### Environment Variables (inherited from Spec-1)

| Variable | Required | Description |
|----------|----------|-------------|
| `COHERE_API_KEY` | Yes | Cohere API key for query embeddings |
| `QDRANT_URL` | Yes | Qdrant Cloud URL |
| `QDRANT_API_KEY` | Yes | Qdrant API key |

### Payload Schema (from Spec-1 implementation)

The existing vectors in `rag_embedding` have this payload structure:

```python
{
    "source_url": str,        # Full URL of source page
    "page_title": str,        # Page title
    "section_heading": str,   # Section heading (may be None)
    "chunk_index": int,       # 0-based position in document
    "chunk_text": str,        # The text content
    "content_hash": str,      # SHA-256 hash for idempotency
    "indexed_at": str,        # ISO timestamp
}
```

**Note**: Filter fields are `source_url`, `page_title`, `section_heading` (not `url`, `title`, `section`).

## Phase Outputs Reference

- **Phase 0**: `research.md` - Design decisions and best practices
- **Phase 1**: `data-model.md`, `contracts/`, `quickstart.md`
- **Phase 2**: `tasks.md` (created by `/sp.tasks`, not `/sp.plan`)

## Complexity Tracking

> No constitution violations - this section is not applicable.
