# Feature Specification: Retrieval Pipeline + Validation

**Feature Branch**: `005-spec02-retrieval-validation`
**Created**: 2025-12-15
**Status**: Draft → Refined
**Reviewed**: 2025-12-15 (spec-2-retrieval-reviewer)
**Input**: User description: "Retrieve relevant chunks from Qdrant (ingested from the deployed Docusaurus book) and validate the end-to-end retrieval pipeline works reliably before adding the agent/UI."

---

## Overview

This feature enables backend developers to retrieve semantically relevant content chunks from the Qdrant vector database (populated by Spec-1 ingestion pipeline) and validate retrieval correctness, stability, and performance. The retrieval interface provides similarity search with metadata filtering, returning top-k chunks suitable for RAG citation use cases.

**Target Users**: Backend developers implementing and verifying RAG retrieval over the book content.

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query Retrieval with Source Attribution (Priority: P1)

A backend developer queries the retrieval system with a natural language question and receives the most relevant chunks from the ingested Docusaurus book content, complete with source metadata (source_url, page_title, section_heading) suitable for citation.

**Why this priority**: This is the core functionality - without accurate retrieval with source attribution, the RAG pipeline cannot function.

**Independent Test**: Can be fully tested by submitting a sample query and verifying that returned chunks are semantically relevant and contain proper citation metadata.

**Acceptance Scenarios**:

1. **Given** the Qdrant collection contains ingested book chunks, **When** a developer submits a natural language query, **Then** the system returns top-k chunks ranked by semantic similarity with source_url, page_title, and section_heading metadata.
2. **Given** a query is submitted, **When** results are returned, **Then** each chunk includes chunk_index and all payload fields required for citation (source_url, page_title, section_heading, chunk_text).
3. **Given** the same query is submitted multiple times, **When** results are returned, **Then** the results are deterministic (identical ordering, scores, and content).

---

### User Story 2 - Metadata-Filtered Search (Priority: P2)

A backend developer retrieves chunks filtered by specific metadata criteria (e.g., restrict results to a particular URL/page or section) to narrow search scope.

**Why this priority**: Metadata filtering enables precise retrieval for specific documentation sections, improving relevance for targeted queries.

**Independent Test**: Can be tested by applying metadata filters and verifying only matching chunks are returned.

**Acceptance Scenarios**:

1. **Given** a query with source_url filter, **When** results are returned, **Then** only chunks from the specified URL(s) appear in results.
2. **Given** a query with section_heading filter, **When** results are returned, **Then** only chunks from the specified section(s) appear in results.
3. **Given** multiple filter criteria, **When** applied together, **Then** results satisfy all filter conditions (AND logic).
4. **Given** a filter with no matching documents, **When** results are returned, **Then** an empty result set is returned (not an error).

---

### User Story 3 - Pipeline Validation and Health Checks (Priority: P2)

A backend developer runs automated validation checks to verify the retrieval pipeline is operational: Qdrant connectivity, collection presence, payload schema correctness, and basic retrieval functionality.

**Why this priority**: Validation ensures the pipeline is healthy before integration, preventing silent failures in production.

**Independent Test**: Can be tested by running the validation suite and verifying all checks pass/fail appropriately.

**Acceptance Scenarios**:

1. **Given** Qdrant is accessible, **When** connectivity check runs, **Then** it reports success with connection details (cluster info, latency).
2. **Given** the target collection exists, **When** collection check runs, **Then** it confirms presence and reports vector count and configuration.
3. **Given** vectors have expected payload schema, **When** schema validation runs, **Then** it verifies required fields (source_url, page_title, section_heading, chunk_index, chunk_text) are present on sampled vectors.
4. **Given** all checks complete, **When** validation suite finishes, **Then** a summary report shows pass/fail status for each check with timing.

---

### User Story 4 - Evaluation Report Generation (Priority: P3)

A backend developer generates an evaluation report showing retrieval quality metrics across sample queries, including pass/fail status, sample queries used, and returned results for manual review.

**Why this priority**: Provides evidence of retrieval quality and supports debugging/tuning before UI integration.

**Independent Test**: Can be tested by running the evaluation harness and reviewing the generated report.

**Acceptance Scenarios**:

1. **Given** a set of predefined sample queries (golden test set), **When** evaluation runs, **Then** a report is generated showing each query, returned chunks, scores, and expected vs actual comparison.
2. **Given** evaluation completes, **When** report is generated, **Then** it includes overall pass/fail status, summary statistics (MRR, precision@k), and timing metrics.

---

### Edge Cases

| Scenario | Expected Behavior | Error Code |
|----------|-------------------|------------|
| Qdrant unreachable | Return `QdrantConnectionError` with retry guidance | E001 |
| Collection does not exist | Return `CollectionNotFoundError` with collection name | E002 |
| Query returns no results | Return empty `SearchResult` (not an error) | - |
| Metadata filter matches nothing | Return empty `SearchResult` (not an error) | - |
| k exceeds available vectors | Return all available vectors (up to total count) | - |
| Empty query string | Return `InvalidQueryError` | E003 |
| Query exceeds max length | Return `InvalidQueryError` with length limit | E003 |
| Cohere API failure | Return `EmbeddingError` with API error details | E004 |
| Invalid filter field name | Return `InvalidFilterError` with valid field names | E005 |
| Qdrant timeout | Return `QdrantTimeoutError` after configured timeout | E006 |

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a retrieval interface that accepts a natural language query and returns top-k semantically similar chunks.
- **FR-002**: System MUST include source metadata (source_url, page_title, section_heading, chunk_index) with each returned chunk for citation purposes.
- **FR-003**: System MUST support configurable k (number of results) with default k=5, range 1-100.
- **FR-004**: System MUST support metadata filtering by source_url, page_title, and section_heading fields using Qdrant filter syntax.
- **FR-005**: System MUST produce deterministic results for identical queries (same input → same output ordering and scores).
- **FR-006**: System MUST use Cohere embed-english-v3.0 with `input_type="search_query"` for query embedding (matching Spec-1 document embedding compatibility).
- **FR-007**: System MUST provide connectivity validation that checks Qdrant cluster accessibility and reports latency.
- **FR-008**: System MUST provide collection validation that confirms target collection exists and reports vector count and configuration.
- **FR-009**: System MUST provide schema validation that verifies payload fields match expected structure by sampling vectors.
- **FR-010**: System MUST generate an evaluation report with pass/fail status, sample queries, returned results, and timing metrics.
- **FR-011**: System MUST handle errors gracefully with typed exceptions and error codes per the error taxonomy.

### Interface Contracts

#### RetrievalRequest

```
RetrievalRequest:
  query: str                          # Natural language query (1-2000 chars)
  top_k: int = 5                      # Number of results (1-100)
  score_threshold: float | None       # Minimum similarity score (0.0-1.0)
  filters: dict | None                # Qdrant filter conditions
    - source_url: str | list[str]     # Filter by URL(s)
    - page_title: str | list[str]     # Filter by title(s)
    - section_heading: str | list[str] # Filter by section(s)
```

#### ChunkResult

```
ChunkResult:
  id: str                             # Qdrant point ID
  score: float                        # Similarity score (0.0-1.0)
  chunk_text: str                     # The text content
  source_url: str                     # Source page URL
  page_title: str                     # Page title
  section_heading: str                # Section heading
  chunk_index: int                    # Position in document
```

#### SearchResult

```
SearchResult:
  query: str                          # Original query
  results: list[ChunkResult]          # Ranked results
  total_found: int                    # Total matches before top_k limit
  latency_ms: float                   # Query execution time
  embedding_ms: float                 # Cohere embedding time
  search_ms: float                    # Qdrant search time
```

#### ValidationReport

```
ValidationReport:
  timestamp: datetime
  checks:
    - connectivity: ValidationResult  # Qdrant reachable
    - collection: ValidationResult    # Collection exists
    - schema: ValidationResult        # Payload schema valid
    - retrieval: ValidationResult     # Basic query works
  overall_status: "pass" | "fail"
  total_duration_ms: float

ValidationResult:
  name: str
  status: "pass" | "fail" | "skip"
  message: str
  duration_ms: float
  details: dict | None
```

#### EvaluationReport

```
EvaluationReport:
  timestamp: datetime
  queries_evaluated: int
  results:
    - query: str
    - expected_urls: list[str] | None  # Expected results (if golden set)
    - actual_results: list[ChunkResult]
    - metrics:
        - hit_at_1: bool
        - hit_at_5: bool
        - mrr: float
  summary:
    - total_queries: int
    - queries_passed: int
    - mrr_average: float
    - precision_at_5: float
  overall_status: "pass" | "fail"
  pass_threshold: float               # MRR threshold for pass (default 0.5)
```

### Key Entities

- **Query**: A natural language search string (1-2000 characters).
- **Chunk**: A text segment from the ingested Docusaurus book with associated metadata (source_url, page_title, section_heading, chunk_index, chunk_text).
- **SearchResult**: A ranked list of chunks with similarity scores, metadata, and timing breakdown.
- **ValidationReport**: A structured output showing health check results with pass/fail status and details.
- **EvaluationReport**: A structured output showing sample query results, relevance metrics, and pass/fail summary.

---

## Error Taxonomy

| Code | Exception Class | Condition | Retry? |
|------|-----------------|-----------|--------|
| E001 | `QdrantConnectionError` | Cannot connect to Qdrant cluster | Yes (3x) |
| E002 | `CollectionNotFoundError` | Target collection does not exist | No |
| E003 | `InvalidQueryError` | Query empty, too long, or malformed | No |
| E004 | `EmbeddingError` | Cohere API failure or timeout | Yes (2x) |
| E005 | `InvalidFilterError` | Unknown filter field or invalid syntax | No |
| E006 | `QdrantTimeoutError` | Search exceeded timeout threshold | Yes (1x) |
| E007 | `SchemaValidationError` | Payload missing required fields | No |
| E008 | `RateLimitError` | Cohere or Qdrant rate limit hit | Yes (backoff) |
| E009 | `ConfigurationError` | Missing env vars or invalid config | No |
| E010 | `InternalError` | Unexpected system error | No |

All exceptions inherit from `RetrievalError` base class with:
- `code: str` - Error code (E001-E010)
- `message: str` - Human-readable message
- `details: dict | None` - Additional context
- `retry_after: int | None` - Seconds to wait before retry (if applicable)

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

| ID | Criterion | Threshold | Measurement Method |
|----|-----------|-----------|-------------------|
| SC-001 | Query latency (p95) | < 2000ms | Time from query submission to results returned |
| SC-002 | Embedding latency (p95) | < 500ms | Cohere API call duration |
| SC-003 | Search latency (p95) | < 200ms | Qdrant search call duration |
| SC-004 | Determinism | 100% | Same query returns identical results (order, scores) |
| SC-005 | Filter accuracy | 100% | Filtered results contain only matching metadata |
| SC-006 | Validation suite duration | < 10s | Total time for all health checks |
| SC-007 | Golden test MRR | ≥ 0.5 | Mean Reciprocal Rank across test queries |
| SC-008 | Schema compliance | 100% | All sampled vectors have required payload fields |
| SC-009 | Error handling coverage | 100% | All error codes have corresponding test cases |
| SC-010 | Health check accuracy | 100% | Checks correctly identify healthy/unhealthy states |

---

## Constraints Catalog

### Technical Constraints (from Spec-1)

| Constraint | Value | Source |
|------------|-------|--------|
| Collection name | `rag_embedding` | Spec-1 |
| Vector dimension | 1024 | Cohere embed-english-v3.0 |
| Distance metric | Cosine | Spec-1 configuration |
| Embedding model | `embed-english-v3.0` | Spec-1 |
| Query input_type | `search_query` | Cohere API (NOT `search_document`) |

### Payload Schema (from Spec-1)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| source_url | string | Yes | Full URL of source page |
| page_title | string | Yes | Page title |
| section_heading | string | Yes | Section heading (or "Introduction") |
| chunk_index | integer | Yes | 0-based position in document |
| chunk_text | string | Yes | The text content |

### Operational Constraints

| Constraint | Value |
|------------|-------|
| Query timeout | 5000ms total |
| Qdrant timeout | 3000ms |
| Cohere timeout | 2000ms |
| Max retries (connection) | 3 |
| Max retries (embedding) | 2 |
| Retry backoff | Exponential (100ms, 200ms, 400ms) |

### Input Constraints

| Constraint | Value |
|------------|-------|
| Query min length | 1 character |
| Query max length | 2000 characters |
| k min value | 1 |
| k max value | 100 |
| k default value | 5 |
| score_threshold range | 0.0 - 1.0 |

---

## Test Strategy

### Golden Test Set Requirements

The evaluation harness MUST include a golden test set with:

1. **Minimum 15 queries** covering:
   - Factual questions (e.g., "What is ROS 2?")
   - Conceptual questions (e.g., "How does navigation work?")
   - Procedural questions (e.g., "How to launch Gazebo simulation?")

2. **Expected results** for each query:
   - At least 1 expected source_url per query
   - Optional: expected chunk_text snippets

3. **Diversity criteria**:
   - Queries span at least 5 different source URLs
   - Queries cover at least 3 different sections

### Test Matrix

| Category | Test Cases | Priority |
|----------|------------|----------|
| Basic retrieval | 15 | P1 |
| Metadata filtering | 12 | P2 |
| Edge cases (errors) | 10 | P1 |
| Determinism | 5 | P1 |
| Performance | 8 | P2 |
| Validation checks | 12 | P2 |
| Filter combinations | 10 | P3 |
| **Total** | **72** | - |

### Repeatability Requirements

- All tests MUST be deterministic
- Tests MUST NOT depend on external state changes
- Tests MUST include timing assertions with tolerance (±20%)
- Tests MUST clean up any state they create

---

## Assumptions

- The Qdrant collection (`rag_embedding`) has been populated by the Spec-1 ingestion pipeline and contains vectors with the expected payload schema.
- Cohere embed-english-v3.0 model is used for query embedding with `input_type="search_query"` (different from `search_document` used in ingestion).
- Qdrant Cloud credentials (QDRANT_URL, QDRANT_API_KEY) are available via environment variables.
- Cohere API key (COHERE_API_KEY) is available via environment variables.
- The retrieval interface is a programmatic API/function, not a user-facing UI (UI is out of scope).
- Cosine similarity is used (matching Spec-1 configuration).

---

## Out of Scope

- User-facing UI or chat interface (covered by future specs).
- Agent/LLM integration for answer generation.
- Write operations to Qdrant (ingestion is Spec-1).
- Performance optimization beyond basic latency validation.
- Advanced relevance tuning, reranking, or hybrid search.
- Authentication/authorization for the retrieval interface.

---

## Dependencies

| Dependency | Type | Description |
|------------|------|-------------|
| Spec-1 (004-Spec01-embedding-pipeline) | Feature | Qdrant collection must be populated with book content vectors |
| Cohere API | External | Required for generating query embeddings |
| Qdrant Cloud | External | Target vector database for retrieval operations |
| Python 3.11+ | Runtime | Execution environment |
| qdrant-client | Library | Qdrant SDK |
| cohere | Library | Cohere SDK |

---

## Acceptance Checklist

| # | Criterion | Status |
|---|-----------|--------|
| 1 | All interface contracts defined | ✅ |
| 2 | All error codes documented | ✅ |
| 3 | All success criteria measurable | ✅ |
| 4 | Payload schema aligned with Spec-1 | ✅ |
| 5 | Test strategy defined | ✅ |
| 6 | Golden test set requirements specified | ✅ |
| 7 | Constraints catalog complete | ✅ |
| 8 | Dependencies documented | ✅ |
| 9 | No implementation details in spec | ✅ |
| 10 | Ready for /sp.plan | ✅ |
