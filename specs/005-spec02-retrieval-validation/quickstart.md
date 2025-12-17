# Quickstart: Retrieval Pipeline + Validation

**Feature**: 005-spec02-retrieval-validation
**Date**: 2025-12-15

## Prerequisites

- Python 3.11+ installed
- `uv` package manager installed
- Spec-1 ingestion completed (vectors exist in Qdrant)
- Environment variables configured

## Setup

### 1. Navigate to backend directory

```bash
cd backend
```

### 2. Install dependencies

```bash
uv sync
```

This installs all dependencies including new ones for Spec-2:
- `pytest` (testing)

### 3. Configure environment

Ensure `.env` file exists with:

```bash
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
```

## Basic Usage

### Perform a search

```python
from retrieval import retrieve
from models import RetrievalRequest

# Simple query
request = RetrievalRequest(query="What is ROS 2?")
result = retrieve(request)

# Print results
for chunk in result.results:
    print(f"[{chunk.score:.3f}] {chunk.page_title}")
    print(f"  URL: {chunk.source_url}")
    print(f"  Text: {chunk.chunk_text[:100]}...")
    print()

# Timing info
print(f"Total: {result.latency_ms:.0f}ms")
print(f"  Embedding: {result.embedding_ms:.0f}ms")
print(f"  Search: {result.search_ms:.0f}ms")
```

### Search with filters

```python
# Filter by URL
request = RetrievalRequest(
    query="navigation",
    top_k=10,
    filters={"source_url": "https://example.com/docs/module-1/"}
)

# Filter by multiple sections
request = RetrievalRequest(
    query="setup instructions",
    filters={"section_heading": ["Setup", "Installation", "Getting Started"]}
)

# Combine filters
request = RetrievalRequest(
    query="simulation",
    filters={
        "page_title": "Gazebo",
        "section_heading": "Configuration"
    }
)
```

### Apply score threshold

```python
# Only return results with similarity >= 0.7
request = RetrievalRequest(
    query="robot control",
    score_threshold=0.7
)
```

## CLI Usage

### Basic search

```bash
python -m retrieval "What is ROS 2?"
```

### With options

```bash
# More results
python -m retrieval "navigation" --top-k 10

# Filter by URL
python -m retrieval "setup" --filter-url "https://example.com/docs/"

# JSON output
python -m retrieval "robot" --json
```

## Validation

### Run validation suite

```python
from validation import validate

report = validate()
print(f"Overall: {report.overall_status}")

for name, check in report.checks.items():
    emoji = "✓" if check.status == "pass" else "✗"
    print(f"  {emoji} {name}: {check.message}")
```

### CLI validation

```bash
python -m retrieval --validate
```

**Expected output**:

```
Validation Report
=================
✓ connectivity: Connected to Qdrant cluster (45ms)
✓ collection: Collection 'rag_embedding' exists with 156 vectors
✓ schema: All required payload fields present
✓ retrieval: Basic search returned 5 results

Overall: PASS (4/4 checks passed)
```

## Evaluation

### Run evaluation with golden test set

```python
from evaluation import evaluate

report = evaluate()
print(f"MRR: {report.summary.mrr_average:.3f}")
print(f"Hit@1: {report.summary.hit_at_1_rate:.1%}")
print(f"Hit@5: {report.summary.hit_at_5_rate:.1%}")
print(f"Overall: {report.overall_status}")
```

### CLI evaluation

```bash
python -m retrieval --evaluate
```

**Expected output**:

```
Evaluation Report
=================
Queries evaluated: 15
MRR: 0.723
Hit@1: 53.3%
Hit@5: 93.3%

Overall: PASS (MRR 0.723 >= 0.500 threshold)
```

### Custom threshold

```bash
python -m retrieval --evaluate --threshold 0.6
```

## Error Handling

```python
from retrieval import retrieve
from errors import (
    InvalidQueryError,
    EmbeddingError,
    QdrantConnectionError,
)

try:
    result = retrieve(request)
except InvalidQueryError as e:
    print(f"Bad request: {e.message}")
except EmbeddingError as e:
    print(f"Embedding failed: {e.message}")
    if e.retry_after:
        print(f"Retry after {e.retry_after}s")
except QdrantConnectionError as e:
    print(f"Connection failed: {e.message}")
```

## Running Tests

### Unit tests

```bash
# All tests
pytest

# Specific module
pytest tests/test_retrieval.py

# Verbose
pytest -v

# With coverage
pytest --cov=backend
```

### Integration tests (requires live services)

```bash
pytest tests/integration/ -v
```

## Troubleshooting

### "Collection not found"

Ensure Spec-1 ingestion was run successfully:

```bash
python main.py  # Run ingestion pipeline
```

### "Cohere API error"

Check API key is valid and has quota remaining:

```bash
echo $COHERE_API_KEY
```

### "Connection timeout"

Verify Qdrant URL and API key:

```bash
curl -H "api-key: $QDRANT_API_KEY" "$QDRANT_URL/collections"
```

### "No results returned"

1. Check collection has vectors: `python -m retrieval --validate`
2. Try broader query without filters
3. Lower score_threshold or remove it

## Next Steps

1. Run validation to ensure pipeline health: `--validate`
2. Run evaluation to verify retrieval quality: `--evaluate`
3. Integrate with your RAG application using `retrieve()` function
