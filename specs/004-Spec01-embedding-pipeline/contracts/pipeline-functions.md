# Pipeline Function Contracts

**Feature**: 004-Spec01-embedding-pipeline
**Date**: 2025-12-14

**ADR Reference**: [ADR-0005: Idempotency Strategy](../../history/adr/0005-idempotency-strategy-via-content-hashing.md)
This document defines the function signatures and contracts for main.py.

---

## Function: get_all_urls

Discover all public URLs from the Docusaurus sitemap.

### Signature

```python
def get_all_urls(base_url: str) -> list[str]:
    """Fetch and parse sitemap.xml to discover all page URLs."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| base_url | str | Must be valid HTTPS URL |

| Output | Type | Guarantee |
|--------|------|-----------|
| urls | list[str] | All URLs are absolute, deduplicated |

### Error Handling

- HTTP errors: Raise ValueError with status code
- Parse errors: Raise ValueError with details
- Empty sitemap: Return empty list (log warning)

---

## Function: extract_text_from_url

Fetch HTML and extract clean text content from a single URL.

### Signature

```python
def extract_text_from_url(url: str) -> tuple[str, str, list[str]]:
    """Fetch page HTML and extract clean text content."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| url | str | Must be valid HTTPS URL |

| Output | Type | Guarantee |
|--------|------|-----------|
| title | str | Non-empty string |
| text_content | str | Normalized whitespace, no HTML tags |
| section_headings | list[str] | May be empty |

### Extraction Rules

1. Primary selector: article.markdown or .theme-doc-markdown
2. Fallback: main element
3. Strip: nav, aside, .sidebar, .pagination-nav, .breadcrumbs
4. Preserve: pre, table, blockquote
5. Title: h1 > title > OpenGraph og:title

---

## Function: chunk_text

Split text into overlapping chunks for embedding.

### Signature

```python
def chunk_text(text: str, chunk_size: int = 2000, chunk_overlap: int = 200) -> list[dict]:
    """Split text into chunks with overlap."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| text | str | Non-empty string |
| chunk_size | int | Must be > chunk_overlap |
| chunk_overlap | int | Must be >= 0 |

| Output | Type | Guarantee |
|--------|------|-----------|
| chunks | list[dict] | Each chunk has text, char_start, char_end, chunk_index |

### Chunking Rules

1. Split on paragraph boundaries first (\n\n)
2. Split on sentence boundaries if chunk too large (. )
3. Hard split at chunk_size if no boundary found
4. Overlap previous N characters at chunk start
5. Minimum chunk size: 100 characters (merge smaller with adjacent)

---

## Function: compute_content_hash

Compute SHA-256 hash of normalized text for change detection (ADR-0005).

### Signature

```python
def compute_content_hash(text: str) -> str:
    """Compute SHA-256 hash of text for idempotency checks."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| text | str | Any string (including empty) |

| Output | Type | Guarantee |
|--------|------|-----------|
| hash | str | 64-character lowercase hex string |

### Normalization Rules

1. Strip leading/trailing whitespace
2. Normalize internal whitespace (collapse multiple spaces/newlines)
3. Encode as UTF-8 before hashing

---

## Function: compute_point_id

Generate deterministic point ID for Qdrant (ADR-0005).

**Note**: Qdrant only accepts UUIDs or 64-bit integers as point IDs, not arbitrary strings.

### Signature

```python
def compute_point_id(source_url: str, chunk_index: int) -> str:
    """Generate deterministic UUID point ID from URL and chunk index."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| source_url | str | Valid URL string |
| chunk_index | int | Non-negative integer |

| Output | Type | Guarantee |
|--------|------|-----------|
| point_id | str | Valid UUID string (e.g., "a1b2c3d4-e5f6-7890-abcd-ef1234567890") |

### Algorithm

```python
import hashlib
import uuid

def compute_point_id(source_url: str, chunk_index: int) -> str:
    raw = f"{source_url}|{chunk_index}"
    hash_bytes = hashlib.sha256(raw.encode()).digest()[:16]  # 16 bytes = 128 bits
    return str(uuid.UUID(bytes=hash_bytes))
```

### Why UUID Format?

- Qdrant requires point IDs to be either UUIDs or 64-bit unsigned integers
- Arbitrary hex strings are NOT supported
- Using first 16 bytes of SHA-256 provides 128-bit collision resistance
- For ~200 items, collision probability is effectively zero (~10^-35)

---

## Function: check_content_changed

Check if content has changed since last indexing (ADR-0005).

### Signature

```python
def check_content_changed(point_id: str, new_content_hash: str) -> bool:
    """Check if content hash differs from stored value in Qdrant."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| point_id | str | Valid UUID string |
| new_content_hash | str | 64-character hex string |

| Output | Type | Guarantee |
|--------|------|-----------|
| changed | bool | True if content changed or point does not exist |

### Logic

1. Query Qdrant for existing point by ID
2. If point does not exist, return True (needs embedding)
3. Compare stored content_hash with new_content_hash
4. Return True if hashes differ, False if identical

### Performance Note

- Adds ~50ms latency per chunk for Qdrant retrieve call
- Saves Cohere API calls when content unchanged
- Net positive for re-runs with mostly unchanged content

---

## Function: embed

Generate embeddings using Cohere API.

### Signature

```python
def embed(texts: list[str]) -> list[list[float]]:
    """Generate embeddings for a batch of texts using Cohere."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| texts | list[str] | 1-96 non-empty strings |

| Output | Type | Guarantee |
|--------|------|-----------|
| embeddings | list[list[float]] | Same length as input, 1024 dims each |

### API Configuration

- Model: embed-english-v3.0
- Input type: search_document
- Truncation: END
- Retry: Exponential backoff, max 3 attempts

---

## Function: create_collection

Ensure Qdrant collection exists with correct configuration.

### Signature

```python
def create_collection(collection_name: str = "rag_embedding") -> bool:
    """Create Qdrant collection if it does not exist."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| collection_name | str | Non-empty string |

| Output | Type | Guarantee |
|--------|------|-----------|
| created | bool | Collection exists after call |

### Collection Configuration

- Vector size: 1024
- Distance: Cosine
- HNSW m: 16
- HNSW ef_construct: 100

---

## Function: save_chunk_to_qdrant

Upsert a single chunk vector to Qdrant.

### Signature

```python
def save_chunk_to_qdrant(point_id: str, vector: list[float], payload: dict) -> bool:
    """Upsert a vector point to Qdrant collection."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| point_id | str | Valid UUID string |
| vector | list[float] | 1024 dimensions |
| payload | dict | Required: source_url, page_title, chunk_index, chunk_text, content_hash, indexed_at |

---

## Function: main

Orchestrate the full pipeline execution.

### Signature

```python
def main() -> dict:
    """Execute the full embedding pipeline."""
```

### Workflow (Updated per ADR-0005)

1. Load environment variables
2. Initialize Qdrant client with timeout (30s)
3. Ensure collection exists (create_collection)
4. Discover URLs from sitemap (get_all_urls)
5. For each URL:
   - Extract text and metadata (extract_text_from_url)
   - Chunk text (chunk_text)
   - For each chunk:
     - Compute point_id (compute_point_id)
     - Compute content_hash (compute_content_hash)
     - Check if changed (check_content_changed)
     - If changed: embed and upsert
     - If unchanged: increment vectors_unchanged counter
   - Log progress: [i/N] Processing: url
6. Generate and return run report

### Statistics Tracked

| Metric | Description |
|--------|-------------|
| urls_discovered | Total URLs from sitemap |
| urls_attempted | URLs processing was attempted |
| urls_successful | URLs processed without error |
| urls_failed | URLs that failed processing |
| chunks_created | Total chunks generated |
| vectors_upserted | Chunks that were embedded and stored |
| vectors_unchanged | Chunks skipped (hash matched) |

### Environment Variables Required

- DOCUSAURUS_BASE_URL
- COHERE_API_KEY
- QDRANT_URL
- QDRANT_API_KEY

### Exit Codes

- 0: Success (all URLs processed or within error threshold)
- 1: Failure (>5% URLs failed or critical error)
