# Research: Embedding Pipeline Design Decisions

**Feature**: 004-Spec01-embedding-pipeline
**Date**: 2025-12-14
**Status**: Complete

## Overview

This document captures the research and design decisions for the Docusaurus-to-Qdrant embedding pipeline. All NEEDS CLARIFICATION items from the Technical Context have been resolved.

---

## Decision 1: Chunk Size and Overlap Strategy

### Decision
- **Chunk Size**: ~500 tokens (approximately 2000 characters)
- **Overlap**: 50 tokens between consecutive chunks
- **Strategy**: Recursive character text splitter with semantic boundaries

### Rationale
- 500 tokens balances context preservation with embedding model efficiency
- Cohere's `embed-english-v3.0` has a max input of 512 tokens; 500 provides safety margin
- 50-token overlap (10%) prevents context loss at chunk boundaries
- Recursive splitting respects paragraph/sentence boundaries for semantic coherence

### Alternatives Considered
1. **Fixed character split (1000 chars)**: Rejected - breaks mid-word/sentence
2. **Sentence-based chunking**: Rejected - too variable in size, inefficient batching
3. **No overlap**: Rejected - loses context at boundaries, hurts retrieval quality

---

## Decision 2: Cohere Embedding Model Selection

### Decision
- **Model**: `embed-english-v3.0`
- **Dimensions**: 1024
- **Input Type**: `search_document` for ingestion, `search_query` for retrieval

### Rationale
- `embed-english-v3.0` is Cohere's latest English embedding model (Nov 2023)
- 1024 dimensions provides excellent semantic capture without excessive storage
- Native support for asymmetric search (document vs query embeddings)
- Free tier supports 100 API calls/minute - sufficient for ~47 URL batch processing

### Alternatives Considered
1. **embed-multilingual-v3.0**: Rejected - English-only content, unnecessary overhead
2. **embed-english-light-v3.0**: Rejected - lower quality for same API cost
3. **OpenAI ada-002**: Rejected - higher cost, no free tier

---

## Decision 3: Qdrant Collection Configuration

### Decision
- **Collection Name**: `rag_embedding` (user-specified)
- **Distance Metric**: Cosine similarity
- **Vector Size**: 1024 (matches Cohere model)
- **HNSW Config**: Default (m=16, ef_construct=100)

### Rationale
- Cosine similarity is standard for normalized embeddings (Cohere outputs normalized vectors)
- Qdrant defaults for HNSW are well-tuned for collections under 1M vectors
- Collection auto-created if missing for self-bootstrapping pipeline

### Alternatives Considered
1. **Dot product distance**: Rejected - requires manual normalization
2. **Euclidean distance**: Rejected - less intuitive for semantic similarity
3. **Custom HNSW tuning**: Rejected - premature optimization for <1000 vectors

---

## Decision 4: Deterministic Point ID Generation

### Decision
- **Algorithm**: UUID derived from `SHA-256(source_url + "|" + chunk_index)`
- **Format**: 32 hex characters (truncated SHA-256)

### Rationale
- Deterministic IDs enable idempotent upserts (same content = same ID)
- URL + chunk_index combination is unique per chunk
- SHA-256 provides collision-resistant hashing
- 32 hex chars sufficient for ~47 URLs × ~5 chunks/page = ~235 unique IDs

### Alternatives Considered
1. **Random UUIDs**: Rejected - creates duplicates on re-run
2. **Sequential integers**: Rejected - not portable across runs
3. **URL-only hash**: Rejected - doesn't distinguish chunks within page

---

## Decision 5: Content Hash for Change Detection

### Decision
- **Algorithm**: SHA-256 of normalized extracted text
- **Storage**: `content_hash` field in Qdrant payload
- **Behavior**: Skip embedding regeneration if hash unchanged

### Rationale
- Enables incremental updates without re-embedding unchanged content
- SHA-256 is fast and collision-resistant
- Stored in payload for comparison during subsequent runs
- Reduces Cohere API calls on re-runs with unchanged content

### Alternatives Considered
1. **MD5 hash**: Rejected - theoretically weaker (though sufficient here)
2. **No change detection**: Rejected - wastes API calls on unchanged content
3. **Last-modified header**: Rejected - not reliably available from all sources

---

## Decision 6: Rate Limiting and Backoff Strategy

### Decision
- **Source Domain**: 2 requests/second rate limit
- **Cohere API**: Exponential backoff starting at 1s, max 3 retries
- **Qdrant API**: 3 retries with 1s delay

### Rationale
- 2 req/s to source is polite crawling, won't trigger WAF
- Cohere free tier: 100 calls/min, backoff handles burst limits
- Qdrant Cloud is reliable; minimal retry needed

### Backoff Formula
```python
delay = min(base_delay * (2 ** attempt), max_delay)
# base_delay=1s, max_delay=30s
```

### Alternatives Considered
1. **No rate limiting**: Rejected - risks IP blocks and API throttling
2. **Fixed delays**: Rejected - inefficient, doesn't adapt to actual limits
3. **Aggressive retries (10+)**: Rejected - delays pipeline unnecessarily

---

## Decision 7: HTML Content Extraction Strategy

### Decision
- **Primary Selector**: `article.markdown` or `.theme-doc-markdown`
- **Fallback**: `main` element content
- **Strip Elements**: `nav`, `aside`, `.sidebar`, `.pagination-nav`, `.breadcrumbs`
- **Preserve**: `pre` (code), `table`, `blockquote`

### Rationale
- Docusaurus uses consistent article structure for main content
- Stripping navigation/sidebar reduces noise in embeddings
- Preserving code blocks enables code search capability
- Fallback handles non-standard pages gracefully

### Alternatives Considered
1. **Full page extraction**: Rejected - includes nav/footer noise
2. **Text-only (no code)**: Rejected - loses valuable technical content
3. **Markdown source fetch**: Rejected - not available from deployed site

---

## Decision 8: Error Handling Philosophy

### Decision
- **Page fetch errors**: Log and skip, continue processing
- **Empty pages**: Log warning, skip vector creation
- **API errors**: Retry with backoff, fail after max retries
- **Threshold**: Fail pipeline if >5% of pages fail

### Rationale
- Graceful degradation ensures partial progress on transient errors
- 5% threshold catches systemic issues while tolerating edge cases
- Detailed error logging enables post-run debugging

### Alternatives Considered
1. **Fail-fast on any error**: Rejected - too brittle for network operations
2. **Ignore all errors**: Rejected - hides systemic problems
3. **100% success required**: Rejected - unrealistic for web scraping

---

## Implementation Notes

### Dependencies (pyproject.toml)

```toml
[project]
dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "httpx>=0.26.0",
    "beautifulsoup4>=4.12.0",
    "lxml>=5.0.0",
    "python-dotenv>=1.0.0",
]
```

### Environment Variables Required

| Variable | Description |
|----------|-------------|
| `DOCUSAURUS_BASE_URL` | `https://hackathon01-docbook-f1h8.vercel.app` |
| `COHERE_API_KEY` | Cohere API key for embeddings |
| `QDRANT_URL` | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Qdrant API key |

### Sitemap Discovery

The target site exposes `sitemap.xml` with 47 URLs:
- Module 01 (ROS2): ~20 pages
- Module 02 (Digital Twin): ~27 pages

All URLs use weekly change frequency with 0.5 priority.

---

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Sitemap structure changes | Fallback to crawl-based discovery |
| Cohere model deprecation | Pin model version, monitor announcements |
| Qdrant schema migration | Version payload schema, document changes |
| Source site downtime | Implement resumable runs with checkpointing |

---

## References

- [Cohere Embed v3 Documentation](https://docs.cohere.com/docs/embed-2)
- [Qdrant Python Client](https://qdrant.tech/documentation/interfaces/python/)
- [Docusaurus Sitemap Plugin](https://docusaurus.io/docs/api/plugins/@docusaurus/plugin-sitemap)
