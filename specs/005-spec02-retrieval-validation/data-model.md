# Data Model: Retrieval Pipeline + Validation

**Feature**: 005-spec02-retrieval-validation
**Date**: 2025-12-15
**Status**: Complete

## Overview

This document defines the data structures used in the retrieval pipeline. All models are implemented as Python dataclasses with full type annotations.

---

## Core Entities

### 1. RetrievalRequest

Request structure for semantic search queries.

```python
@dataclass
class RetrievalRequest:
    """Request for semantic similarity search."""

    query: str                              # Natural language query (1-2000 chars)
    top_k: int = 5                          # Number of results (1-100)
    score_threshold: float | None = None    # Minimum score (0.0-1.0)
    filters: dict[str, str | list[str]] | None = None  # Metadata filters

    # Validation rules:
    # - query: len(query) >= 1 and len(query) <= 2000
    # - top_k: top_k >= 1 and top_k <= 100
    # - score_threshold: None or (0.0 <= score_threshold <= 1.0)
    # - filters.keys() must be in {"source_url", "page_title", "section_heading"}
```

**Filter Structure**:
```python
filters = {
    "source_url": "https://...",              # Single URL
    "source_url": ["url1", "url2"],           # Multiple URLs (OR)
    "page_title": "Introduction",             # Single title
    "section_heading": ["Setup", "Config"],   # Multiple sections (OR)
}
```

---

### 2. ChunkResult

Individual search result with metadata for citations.

```python
@dataclass
class ChunkResult:
    """Single chunk result from retrieval."""

    id: str                    # Qdrant point ID (32-char hex)
    score: float               # Similarity score (0.0-1.0)
    chunk_text: str            # The text content
    source_url: str            # Source page URL
    page_title: str            # Page title
    section_heading: str | None  # Section heading (may be None)
    chunk_index: int           # Position in document (0-based)
```

---

### 3. SearchResult

Complete search response with timing breakdown.

```python
@dataclass
class SearchResult:
    """Complete search result with metadata."""

    query: str                      # Original query
    results: list[ChunkResult]      # Ranked results
    total_found: int                # Total matches before top_k limit
    latency_ms: float               # Total query time
    embedding_ms: float             # Cohere API time
    search_ms: float                # Qdrant search time
```

---

### 4. ValidationResult

Single health check result.

```python
@dataclass
class ValidationResult:
    """Result of a single validation check."""

    name: str                       # Check name (e.g., "connectivity")
    status: Literal["pass", "fail", "skip"]
    message: str                    # Human-readable message
    duration_ms: float              # Check duration
    details: dict[str, Any] | None = None  # Additional context
```

---

### 5. ValidationReport

Complete validation suite report.

```python
@dataclass
class ValidationReport:
    """Complete validation suite report."""

    timestamp: datetime
    checks: dict[str, ValidationResult]  # Check name -> result
    overall_status: Literal["pass", "fail"]
    total_duration_ms: float
```

**Check Names**:
- `connectivity`: Qdrant cluster reachable
- `collection`: Target collection exists
- `schema`: Payload fields valid
- `retrieval`: Basic search works

---

### 6. EvaluationQuery

Single evaluation query with expected results.

```python
@dataclass
class EvaluationQuery:
    """Single query for evaluation."""

    query: str                          # The query text
    expected_urls: list[str]            # Expected source URLs
    query_type: Literal["factual", "conceptual", "procedural"]
```

---

### 7. EvaluationQueryResult

Result for a single evaluation query.

```python
@dataclass
class EvaluationQueryResult:
    """Result for a single evaluation query."""

    query: str
    expected_urls: list[str] | None
    actual_results: list[ChunkResult]
    hit_at_1: bool                      # Correct result at position 1
    hit_at_5: bool                      # Correct result in top 5
    mrr: float                          # Reciprocal rank for this query
    rank: int | None                    # Position of first correct result
```

---

### 8. EvaluationReport

Complete evaluation report with summary statistics.

```python
@dataclass
class EvaluationReport:
    """Complete evaluation report."""

    timestamp: datetime
    queries_evaluated: int
    results: list[EvaluationQueryResult]
    summary: EvaluationSummary
    overall_status: Literal["pass", "fail"]
    pass_threshold: float = 0.5         # MRR threshold for pass

@dataclass
class EvaluationSummary:
    """Summary statistics for evaluation."""

    total_queries: int
    queries_passed: int                  # hit_at_5 == True
    mrr_average: float                   # Mean Reciprocal Rank
    hit_at_1_rate: float                 # % with correct top result
    hit_at_5_rate: float                 # % with correct in top 5
```

---

## Error Models

### RetrievalError (Base)

```python
@dataclass
class RetrievalError(Exception):
    """Base exception for retrieval errors."""

    code: str                           # Error code (E001-E010)
    message: str                        # Human-readable message
    details: dict[str, Any] | None = None
    retry_after: int | None = None      # Seconds to wait (if retryable)
```

### Specific Errors

| Class | Code | Retryable |
|-------|------|-----------|
| `QdrantConnectionError` | E001 | Yes (3x) |
| `CollectionNotFoundError` | E002 | No |
| `InvalidQueryError` | E003 | No |
| `EmbeddingError` | E004 | Yes (2x) |
| `InvalidFilterError` | E005 | No |
| `QdrantTimeoutError` | E006 | Yes (1x) |
| `SchemaValidationError` | E007 | No |
| `RateLimitError` | E008 | Yes (backoff) |
| `ConfigurationError` | E009 | No |
| `InternalError` | E010 | No |

---

## Configuration Constants

```python
# config.py

# Cohere settings
COHERE_MODEL = "embed-english-v3.0"
COHERE_DIMENSIONS = 1024
COHERE_INPUT_TYPE_QUERY = "search_query"     # For retrieval
COHERE_INPUT_TYPE_DOCUMENT = "search_document"  # For ingestion (Spec-1)

# Qdrant settings
QDRANT_COLLECTION = "rag_embedding"

# Timeouts (milliseconds)
QUERY_TIMEOUT_MS = 5000
QDRANT_TIMEOUT_MS = 3000
COHERE_TIMEOUT_MS = 2000

# Retry settings
MAX_RETRIES_CONNECTION = 3
MAX_RETRIES_EMBEDDING = 2
RETRY_BACKOFF_MS = [100, 200, 400]

# Input constraints
QUERY_MIN_LENGTH = 1
QUERY_MAX_LENGTH = 2000
K_MIN = 1
K_MAX = 100
K_DEFAULT = 5

# Validation settings
SCHEMA_SAMPLE_SIZE = 10  # Vectors to sample for schema validation

# Evaluation settings
MRR_PASS_THRESHOLD = 0.5

# Required payload fields (from Spec-1)
REQUIRED_PAYLOAD_FIELDS = {
    "source_url",
    "page_title",
    "section_heading",
    "chunk_index",
    "chunk_text",
}

# Filterable fields
FILTERABLE_FIELDS = {
    "source_url",
    "page_title",
    "section_heading",
}
```

---

## Entity Relationships

```
RetrievalRequest
       │
       ▼
   retrieve()
       │
       ▼
  SearchResult
       │
       ├──→ ChunkResult[] (results)
       │
       └──→ Timing (embedding_ms, search_ms)

EvaluationQuery[]
       │
       ▼
   evaluate()
       │
       ▼
EvaluationReport
       │
       ├──→ EvaluationQueryResult[] (per query)
       │
       └──→ EvaluationSummary (aggregated)

ValidationReport
       │
       └──→ ValidationResult[] (per check)
```

---

## State Transitions

### Search Flow

```
RetrievalRequest
    │
    ├─[validate]──→ InvalidQueryError (if invalid)
    │
    ├─[embed]─────→ EmbeddingError (if Cohere fails)
    │
    ├─[search]────→ QdrantConnectionError / QdrantTimeoutError
    │
    └─[success]───→ SearchResult
```

### Validation Flow

```
ValidationReport
    │
    ├─[connectivity]──→ pass/fail
    │
    ├─[collection]────→ pass/fail/skip (if connectivity failed)
    │
    ├─[schema]────────→ pass/fail/skip (if collection failed)
    │
    └─[retrieval]─────→ pass/fail/skip (if schema failed)
```

---

## JSON Serialization

All dataclasses support JSON serialization via `dataclasses.asdict()`:

```python
import json
from dataclasses import asdict

result = retrieve(request)
json_output = json.dumps(asdict(result), default=str, indent=2)
```

The `default=str` handles datetime serialization.
