# API Contract: Retrieval Pipeline

**Feature**: 005-spec02-retrieval-validation
**Date**: 2025-12-15
**Status**: Complete

## Overview

This document defines the Python API contracts for the retrieval pipeline. The retrieval module exposes functions for semantic search, validation, and evaluation.

---

## Core Functions

### 1. retrieve()

Main entry point for semantic similarity search.

```python
def retrieve(request: RetrievalRequest) -> SearchResult:
    """
    Perform semantic similarity search over the Qdrant collection.

    Args:
        request: RetrievalRequest with query, top_k, filters, score_threshold

    Returns:
        SearchResult with ranked chunks and timing breakdown

    Raises:
        InvalidQueryError: Query empty, too long, or malformed (E003)
        InvalidFilterError: Unknown filter field (E005)
        EmbeddingError: Cohere API failure (E004)
        QdrantConnectionError: Cannot connect to Qdrant (E001)
        QdrantTimeoutError: Search timeout exceeded (E006)
        CollectionNotFoundError: Collection does not exist (E002)

    Example:
        >>> request = RetrievalRequest(query="What is ROS 2?", top_k=5)
        >>> result = retrieve(request)
        >>> print(result.results[0].chunk_text)
        "ROS 2 is the next-generation Robot Operating System..."
    """
```

**Behavior**:
1. Validate request parameters
2. Generate query embedding using Cohere (`input_type="search_query"`)
3. Execute Qdrant search with filters (if provided)
4. Map results to ChunkResult objects
5. Sort by (score DESC, id ASC) for determinism
6. Return SearchResult with timing breakdown

---

### 2. retrieve_with_retry()

Convenience wrapper with automatic retry logic.

```python
def retrieve_with_retry(
    request: RetrievalRequest,
    max_retries: int = 3,
) -> SearchResult:
    """
    Perform retrieval with automatic retry for transient errors.

    Args:
        request: RetrievalRequest with query parameters
        max_retries: Maximum retry attempts for retryable errors

    Returns:
        SearchResult with ranked chunks

    Raises:
        RetrievalError: After all retries exhausted or non-retryable error

    Note:
        Only retries for: E001, E004, E006, E008
        Non-retryable errors: E002, E003, E005, E007, E009, E010
    """
```

---

### 3. validate()

Run the complete validation suite.

```python
def validate() -> ValidationReport:
    """
    Run all health checks and return validation report.

    Returns:
        ValidationReport with pass/fail status for each check

    Checks performed:
        1. connectivity: Can connect to Qdrant cluster
        2. collection: Target collection exists with vectors
        3. schema: Payload fields match expected structure
        4. retrieval: Basic search returns results

    Example:
        >>> report = validate()
        >>> print(report.overall_status)
        "pass"
        >>> for name, result in report.checks.items():
        ...     print(f"{name}: {result.status}")
    """
```

**Check Details**:

| Check | Pass Criteria | Fail Info |
|-------|---------------|-----------|
| connectivity | API responds < 3s | Connection error message |
| collection | Collection exists, vector_count > 0 | Collection name, actual collections |
| schema | All required fields present in sample | Missing fields list |
| retrieval | Search returns ≥ 1 result for test query | Error message |

---

### 4. evaluate()

Run evaluation with golden test set.

```python
def evaluate(
    queries: list[EvaluationQuery] | None = None,
    pass_threshold: float = 0.5,
) -> EvaluationReport:
    """
    Evaluate retrieval quality using golden test set.

    Args:
        queries: List of evaluation queries (default: load from golden_queries.json)
        pass_threshold: MRR threshold for overall pass (default: 0.5)

    Returns:
        EvaluationReport with per-query results and summary statistics

    Example:
        >>> report = evaluate()
        >>> print(f"MRR: {report.summary.mrr_average:.3f}")
        >>> print(f"Hit@5: {report.summary.hit_at_5_rate:.1%}")
    """
```

**Metrics Calculated**:
- **MRR**: Mean Reciprocal Rank (1/position of first correct result)
- **Hit@1**: Was correct result the top result?
- **Hit@5**: Was correct result in top 5?

---

### 5. check_connectivity()

Individual connectivity check.

```python
def check_connectivity() -> ValidationResult:
    """Check if Qdrant cluster is reachable."""
```

### 6. check_collection()

Individual collection check.

```python
def check_collection() -> ValidationResult:
    """Check if target collection exists and has vectors."""
```

### 7. check_schema()

Individual schema validation.

```python
def check_schema(sample_size: int = 10) -> ValidationResult:
    """Validate payload schema by sampling vectors."""
```

### 8. check_retrieval()

Individual retrieval smoke test.

```python
def check_retrieval() -> ValidationResult:
    """Perform basic retrieval test to verify search works."""
```

---

## Filter Building

### build_filter()

Internal function to construct Qdrant filter from request.

```python
def build_filter(
    filters: dict[str, str | list[str]] | None
) -> models.Filter | None:
    """
    Build Qdrant filter from request filters.

    Args:
        filters: Dict mapping field names to values or value lists

    Returns:
        Qdrant Filter object or None if no filters

    Raises:
        InvalidFilterError: If filter field not in FILTERABLE_FIELDS

    Filter Logic:
        - Multiple values for same field: OR
        - Multiple fields: AND
        - Single value: exact match

    Example:
        >>> filters = {"source_url": ["url1", "url2"], "page_title": "Intro"}
        >>> qdrant_filter = build_filter(filters)
        # Returns: (source_url IN [url1, url2]) AND (page_title == "Intro")
    """
```

---

## CLI Interface

### Command: python -m retrieval

```bash
# Basic query
python -m retrieval "What is ROS 2?"

# With options
python -m retrieval "navigation" --top-k 10 --filter-url "https://..."

# Validation
python -m retrieval --validate

# Evaluation
python -m retrieval --evaluate
python -m retrieval --evaluate --threshold 0.6

# JSON output
python -m retrieval "query" --json
```

**CLI Arguments**:

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `query` | positional | - | Search query (required unless --validate/--evaluate) |
| `--top-k` | int | 5 | Number of results |
| `--filter-url` | str | None | Filter by source_url |
| `--filter-title` | str | None | Filter by page_title |
| `--filter-section` | str | None | Filter by section_heading |
| `--score-threshold` | float | None | Minimum similarity score |
| `--validate` | flag | False | Run validation suite |
| `--evaluate` | flag | False | Run evaluation harness |
| `--threshold` | float | 0.5 | MRR pass threshold (with --evaluate) |
| `--json` | flag | False | Output as JSON |

---

## Error Responses

All errors return `RetrievalError` subclasses with structured information:

```python
try:
    result = retrieve(request)
except InvalidQueryError as e:
    print(f"Error {e.code}: {e.message}")
    # E003: Query must be between 1 and 2000 characters
except EmbeddingError as e:
    print(f"Retry after {e.retry_after}s: {e.message}")
```

---

## Performance Contracts

| Operation | p95 Latency | Notes |
|-----------|-------------|-------|
| `retrieve()` | < 2000ms | Total end-to-end |
| Cohere embedding | < 500ms | Single query embedding |
| Qdrant search | < 200ms | With filters |
| `validate()` | < 10s | All 4 checks |
| `evaluate()` | < 60s | 15 queries |

---

## Thread Safety

- All functions are **thread-safe** for read operations
- Clients (Cohere, Qdrant) are initialized lazily and cached per-process
- No shared mutable state between calls

---

## Environment Requirements

```bash
# Required
COHERE_API_KEY=...
QDRANT_URL=https://...
QDRANT_API_KEY=...
```

Missing environment variables raise `ConfigurationError` (E009).
