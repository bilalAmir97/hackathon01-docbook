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

### Signature

```python
def compute_point_id(source_url: str, chunk_index: int) -> str:
    """Generate deterministic point ID from URL and chunk index."""
```

### Contract

| Input | Type | Validation |
|-------|------|------------|
| source_url | str | Valid URL string |
| chunk_index | int | Non-negative integer |

| Output | Type | Guarantee |
|--------|------|-----------|
| point_id | str | 32-character lowercase hex string |

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
| point_id | str | 32-character hex string |
| new_content_hash | str | 64-character hex string |

| Output | Type | Guarantee |
|--------|------|-----------|
| changed | bool | True if content changed or point does not exist |

### Logic

1. Query Qdrant for existing point by ID
2. If point does not exist, return True (needs embedding)
3. Compare stored content_hash with new_content_hash
4. Return True if hashes differ, False if identical

---

