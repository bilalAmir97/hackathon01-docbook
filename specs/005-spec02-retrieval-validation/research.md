# Research: Retrieval Pipeline + Validation

**Feature**: 005-spec02-retrieval-validation
**Date**: 2025-12-15
**Status**: Complete

## Research Questions Addressed

### 1. Query vs Document Embedding Input Types

**Question**: How should query embeddings differ from document embeddings in Cohere?

**Decision**: Use `input_type="search_query"` for retrieval queries.

**Rationale**: Cohere's embed-english-v3.0 model is trained with asymmetric similarity in mind:
- `search_document`: Used when embedding text that will be stored/indexed
- `search_query`: Used when embedding queries to search against stored documents

The asymmetric approach improves retrieval quality because queries are typically shorter and express information needs differently than the full document text.

**Alternatives Considered**:
- Using `search_document` for both (rejected: suboptimal retrieval quality)
- Using `classification` (rejected: wrong use case)

**Source**: Cohere API documentation for embed endpoint

---

### 2. Qdrant Filter Syntax for Metadata

**Question**: What is the correct filter syntax for Qdrant metadata filtering?

**Decision**: Use Qdrant's `models.Filter` with `FieldCondition` for payload filtering.

**Rationale**: Qdrant supports two filter syntaxes:
1. Python SDK models (recommended): Type-safe, IDE support
2. Dict-based JSON filters: More flexible but error-prone

For our use case with specific field filters (source_url, page_title, section_heading), the SDK models provide better validation and clearer code.

**Implementation Pattern**:
```python
from qdrant_client.http import models

filter_conditions = []
if source_url:
    filter_conditions.append(
        models.FieldCondition(
            key="source_url",
            match=models.MatchValue(value=source_url)
        )
    )

query_filter = models.Filter(must=filter_conditions) if filter_conditions else None
```

**Alternatives Considered**:
- Raw dict filters (rejected: no type safety, harder to debug)
- Multiple separate queries (rejected: inefficient)

**Source**: Qdrant Python SDK documentation

---

### 3. Deterministic Result Ordering

**Question**: How to ensure deterministic results for identical queries?

**Decision**: Order by score descending, then by point ID for tie-breaking.

**Rationale**:
- Qdrant search returns results ordered by score
- When scores are equal (rare but possible), results may be non-deterministic
- Adding secondary sort by point_id ensures reproducibility

**Implementation**:
```python
# Post-search sorting
results.sort(key=lambda x: (-x.score, x.id))
```

**Alternatives Considered**:
- Rely solely on Qdrant ordering (rejected: ties may be non-deterministic)
- Add timestamp to sort (rejected: not necessary, point ID sufficient)

---

### 4. Validation Suite Design

**Question**: What health checks are essential for retrieval validation?

**Decision**: Four-check validation suite:

| Check | Purpose | Pass Criteria |
|-------|---------|---------------|
| Connectivity | Verify Qdrant cluster reachable | Successful API call |
| Collection | Verify collection exists | Collection found with vectors |
| Schema | Verify payload structure | All required fields present |
| Retrieval | Verify search works | Results returned for test query |

**Rationale**: These checks cover the critical path from connection to actual retrieval. Each check is independent and provides specific diagnostic information.

**Alternatives Considered**:
- Single "does it work" test (rejected: no diagnostic value)
- Extensive performance benchmarks (rejected: out of scope for validation)

---

### 5. Evaluation Metrics Selection

**Question**: Which retrieval metrics are appropriate for this use case?

**Decision**: Use MRR (Mean Reciprocal Rank) as primary metric, with hit@1 and hit@5 as secondary.

**Rationale**:
- **MRR**: Measures how highly the correct result is ranked (0-1 scale)
- **Hit@1**: Binary - was the top result correct?
- **Hit@5**: Binary - was the correct result in top 5?

These metrics are:
1. Easy to interpret
2. Require minimal ground truth (just expected source_url)
3. Suitable for RAG evaluation where position matters

**MRR Calculation**:
```python
def mrr(ranks: list[int]) -> float:
    return sum(1.0 / r for r in ranks if r > 0) / len(ranks)
```

**Pass Threshold**: MRR ≥ 0.5 (meaning correct results are typically in top 2 positions on average)

**Alternatives Considered**:
- NDCG (rejected: requires graded relevance judgments)
- Recall@k (rejected: need full relevance set)
- Precision@k (rejected: need relevance judgments for all results)

---

### 6. Error Handling and Retry Strategy

**Question**: What retry strategy balances reliability with latency?

**Decision**: Differentiated retry policy based on error type:

| Error Type | Retries | Backoff | Rationale |
|------------|---------|---------|-----------|
| Connection errors | 3 | 100ms, 200ms, 400ms | Transient network issues |
| Embedding errors | 2 | 100ms, 200ms | Cohere rate limits |
| Timeout errors | 1 | None | May indicate overload |
| Validation errors | 0 | N/A | User input errors |

**Rationale**:
- Connection issues are often transient, warrant more retries
- API rate limits benefit from backoff
- Timeouts suggest systemic issues, limited retry value
- Validation errors should fail fast

**Alternatives Considered**:
- Uniform retry for all errors (rejected: wastes time on unrecoverable errors)
- No retries (rejected: too brittle for network operations)
- Circuit breaker pattern (rejected: overkill for single-user CLI)

---

### 7. Test Data Strategy

**Question**: How to create meaningful golden test queries?

**Decision**: Curated golden test set with three query types:

1. **Factual** (5 queries): Specific facts from documentation
   - Example: "What is ROS 2?"

2. **Conceptual** (5 queries): Understanding concepts
   - Example: "How does navigation work in robotics?"

3. **Procedural** (5 queries): How-to questions
   - Example: "How to launch Gazebo simulation?"

**Golden Query Format**:
```json
{
  "query": "What is ROS 2?",
  "expected_urls": ["https://...intro..."],
  "query_type": "factual"
}
```

**Rationale**:
- 15 queries is minimum viable for statistical significance
- Three types ensure coverage of different retrieval scenarios
- Expected URLs provide ground truth for MRR calculation

**Alternatives Considered**:
- Synthetic query generation (rejected: may not reflect real usage)
- User-provided queries only (rejected: need baseline for automated tests)

---

### 8. Module Organization

**Question**: Single file (like Spec-1) or multiple modules?

**Decision**: Multiple modules for separation of concerns.

**Rationale**:
- Spec-1's single-file approach works for one-time scripts
- Retrieval will be used as a library by future specs
- Testing is easier with modular code
- Error handling deserves its own module for reusability

**Module Structure**:
```
backend/
├── main.py          # Spec-1 (unchanged)
├── retrieval.py     # Core search
├── errors.py        # Exception hierarchy
├── validation.py    # Health checks
├── evaluation.py    # Test harness
├── models.py        # Dataclasses
└── config.py        # Constants
```

**Alternatives Considered**:
- Single file (rejected: harder to test, harder to extend)
- Package structure (rejected: overkill for 6 small modules)

---

## Summary

All research questions resolved. Key decisions:
1. Use `input_type="search_query"` for Cohere embeddings
2. Use Qdrant SDK filter models for type safety
3. Sort by (score DESC, id ASC) for determinism
4. Four-check validation suite
5. MRR as primary evaluation metric with 0.5 threshold
6. Differentiated retry strategy per error type
7. 15 curated golden queries (factual/conceptual/procedural)
8. Modular architecture for testability
