# ADR-0005: Idempotency Strategy via Content Hashing

- **Status:** Accepted
- **Date:** 2025-12-14
- **Feature:** 004-Spec01-embedding-pipeline
- **Context:** The embedding pipeline needs to support re-runs without creating duplicate vectors or wasting API calls. When content changes, vectors should be updated; when content is unchanged, embedding regeneration should be skipped to conserve Cohere API quota (free tier: 100 calls/min).

## Decision

Implement a two-layer idempotency strategy:

1. **Deterministic Point IDs**: Generate vector point IDs by converting SHA-256(source_url + "|" + chunk_index) to UUID format. Qdrant requires UUIDs or 64-bit integers (not arbitrary strings). Same URL + chunk position always produces the same ID, enabling upsert-based updates instead of insert + delete.

2. **Content Hash Change Detection**: Compute SHA-256(normalized_text) for each chunk and store in Qdrant payload as content_hash. Before embedding generation:
   - Query Qdrant for existing point by ID
   - Compare stored content_hash with new hash
   - Skip embedding if hashes match (track as vectors_unchanged)
   - Regenerate embedding only if hash differs or point does not exist

3. **Payload Schema**: Store hash alongside vector for comparison:
   - content_hash: 64-char SHA-256 hex string
   - indexed_at: ISO8601 timestamp of last update

## Consequences

### Positive

- **API Cost Reduction**: Re-runs with unchanged content consume zero Cohere API calls, staying within free tier limits
- **Idempotent Upserts**: Same input always produces same state; safe to run multiple times
- **Incremental Updates**: Only changed pages trigger embedding regeneration
- **Auditability**: indexed_at timestamp shows when each vector was last updated
- **Collision-Resistant**: SHA-256 provides strong uniqueness guarantees for both ID and hash

### Negative

- **Storage Overhead**: Each vector payload includes 64-char hash string (~64 bytes per chunk)
- **Query Latency**: Requires Qdrant retrieve call before embedding decision (adds ~50ms per chunk)
- **Complexity**: More logic in main loop compared to blind upsert approach
- **Stale Detection Gap**: Does not detect orphan vectors (pages removed from sitemap)

## Alternatives Considered

### Alternative A: Blind Upsert (No Change Detection)
- **Approach**: Always regenerate embeddings and upsert, relying on deterministic IDs for deduplication
- **Pros**: Simpler implementation, no Qdrant queries needed
- **Cons**: Wastes API calls on unchanged content; with 200 chunks and 100 calls/min limit, re-runs take 2+ minutes even when nothing changed
- **Why rejected**: Unacceptable for iterative development where re-runs are frequent

### Alternative B: Last-Modified Header
- **Approach**: Check HTTP Last-Modified or ETag headers before fetching pages
- **Pros**: No need to store hashes; standard HTTP caching
- **Cons**: Vercel/Docusaurus deployments do not reliably provide these headers; still requires full fetch to compare
- **Why rejected**: Not reliably available from target site

### Alternative C: File-Based Checkpoint
- **Approach**: Store processed URLs and hashes in local JSON file
- **Pros**: No Qdrant queries; works offline
- **Cons**: Checkpoint file can desync from Qdrant state; not portable across machines
- **Why rejected**: Single source of truth should be Qdrant itself

### Alternative D: MD5 Hash
- **Approach**: Use MD5 instead of SHA-256 for content hashing
- **Pros**: Slightly faster hash computation
- **Cons**: Theoretically weaker collision resistance (though sufficient for this use case)
- **Why rejected**: SHA-256 is standard for content addressing; negligible performance difference for ~200 chunks

## References

- Feature Spec: [spec.md](../../specs/004-Spec01-embedding-pipeline/spec.md) - FR-011 (idempotency requirements)
- Implementation Plan: [plan.md](../../specs/004-Spec01-embedding-pipeline/plan.md) - Key Design Decisions #3, #4
- Research: [research.md](../../specs/004-Spec01-embedding-pipeline/research.md) - Decisions 4, 5
- Related ADRs: None (first data pipeline ADR)
- Architecture Review: Identified gap in contracts for hash comparison logic
