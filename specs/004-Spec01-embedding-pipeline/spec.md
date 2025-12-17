# Feature Specification: Embedding Pipeline Setup

**Feature Branch**: `004-Spec01-embedding-pipeline`
**Created**: 2025-12-14
**Status**: Ready for Planning
**Input**: User description: "Extract text from deployed Docusaurus URLs, generate embeddings using Cohere, and store them in Qdrant for RAG-based retrieval."

## Overview

This feature establishes a document ingestion pipeline that extracts text content from a deployed Docusaurus book, transforms it into vector embeddings using Cohere's embedding models, and stores these vectors in Qdrant vector database for retrieval-augmented generation (RAG) use cases.

**Target Audience**: Developers building a RAG backend for retrieval over the published book.

**Core Value**: Enable semantic search and retrieval capabilities over the book content by creating a searchable vector index of all documentation pages.

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Initial Pipeline Execution (Priority: P1)

As a developer, I want to run the embedding pipeline against the deployed Docusaurus book so that all public content becomes searchable via vector similarity.

**Why this priority**: This is the core functionality - without initial ingestion, no RAG retrieval is possible.

**Independent Test**: Can be fully tested by running the pipeline once and verifying that vectors exist in Qdrant with correct metadata.

**Acceptance Scenarios**:

1. **Given** a deployed Docusaurus book with public URLs, **When** I execute the pipeline, **Then** all accessible pages are fetched and processed successfully.
2. **Given** multiple book chapters and sections, **When** the pipeline completes, **Then** each page is represented as one or more vector chunks in Qdrant.
3. **Given** a page with mixed content (text, code blocks, images), **When** the pipeline processes it, **Then** only text content is extracted and embedded (images are excluded, code blocks are preserved as text).

---

### User Story 2 - Re-running Pipeline (Idempotency) (Priority: P1)

As a developer, I want to re-run the pipeline without creating duplicate entries so that I can safely update the vector store when content changes.

**Why this priority**: Critical for production use - content updates require re-indexing without data corruption.

**Independent Test**: Can be tested by running the pipeline twice consecutively and verifying no duplicate vectors are created.

**Acceptance Scenarios**:

1. **Given** vectors already exist in Qdrant from a previous run, **When** I run the pipeline again, **Then** existing vectors are updated (not duplicated).
2. **Given** a page URL that was previously indexed, **When** the same URL is processed again, **Then** its vectors are replaced with the new content.
3. **Given** a page that no longer exists (404), **When** the pipeline encounters it during re-run, **Then** the error is logged and processing continues with other pages.

---

### User Story 3 - Chunk-based Retrieval (Priority: P2)

As a developer querying the RAG system, I want each stored vector to contain source metadata so that I can trace retrieved content back to its original location.

**Why this priority**: Essential for building useful RAG applications - users need to know where information came from.

**Independent Test**: Can be tested by querying Qdrant and verifying each returned vector includes source URL, page title, and chunk position.

**Acceptance Scenarios**:

1. **Given** a page is processed into multiple chunks, **When** stored in Qdrant, **Then** each chunk includes: source URL, page title, chunk index, and section heading (if applicable).
2. **Given** a chunk is retrieved via similarity search, **When** the developer examines metadata, **Then** they can construct a direct link to the original content.

---

### User Story 4 - Pipeline Status and Reporting (Priority: P3)

As a developer, I want to see a summary report after pipeline execution so that I can verify the ingestion completed successfully.

**Why this priority**: Operational visibility - helps troubleshoot issues and confirm successful runs.

**Independent Test**: Can be tested by running the pipeline and verifying a summary report is generated.

**Acceptance Scenarios**:

1. **Given** the pipeline completes, **When** I review the output, **Then** I see: total URLs processed, successful extractions, failed URLs, total chunks created, and total vectors stored.
2. **Given** some URLs fail during processing, **When** the pipeline completes, **Then** failed URLs are listed with their error reasons.

---

### Edge Cases

- What happens when a URL returns a 404 or 5xx error? **System logs the error with URL, skips the page, and continues processing.**
- What happens when a page has no extractable text content? **System logs a warning and skips creating vectors for that page.**
- What happens when Cohere API rate limits are hit? **System implements exponential backoff and retries, with configurable maximum retries.**
- What happens when Qdrant is temporarily unavailable? **System retries with backoff; after max retries, fails gracefully with clear error message.**
- What happens when a page contains only images/media? **System logs that no text was extracted and skips vector creation for that page.**
- What happens when text content exceeds maximum chunk size? **System splits content into multiple chunks with configurable overlap.**

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST discover all public URLs from the deployed Docusaurus book (sitemap or crawling-based discovery).
- **FR-002**: System MUST fetch HTML content from each discovered URL.
- **FR-003**: System MUST extract clean text content from HTML, preserving code blocks as text and removing navigation/boilerplate elements.
- **FR-004**: System MUST split extracted text into chunks suitable for embedding (configurable chunk size with default of ~500 tokens).
- **FR-005**: System MUST maintain text overlap between consecutive chunks to preserve context (configurable overlap, default ~50 tokens).
- **FR-006**: System MUST generate embeddings for each text chunk using Cohere's embedding API.
- **FR-007**: System MUST store vectors in Qdrant with associated metadata (source URL, page title, chunk index, section heading).
- **FR-008**: System MUST use deterministic IDs for vectors based on source URL and chunk index to enable idempotent updates.
- **FR-009**: System MUST handle API errors gracefully with retry logic and exponential backoff.
- **FR-010**: System MUST generate a run report in JSON format with the following schema:
  ```json
  {
    "run_id": "UUID",
    "started_at": "ISO8601",
    "completed_at": "ISO8601",
    "duration_seconds": "number",
    "source_base_url": "string",
    "statistics": {
      "urls_discovered": "number",
      "urls_attempted": "number",
      "urls_successful": "number",
      "urls_failed": "number",
      "chunks_created": "number",
      "vectors_upserted": "number",
      "vectors_unchanged": "number"
    },
    "failures": [
      {"url": "string", "error_code": "string", "error_message": "string"}
    ],
    "configuration": {
      "chunk_size": "number",
      "chunk_overlap": "number",
      "cohere_model": "string",
      "qdrant_collection": "string"
    }
  }
  ```
- **FR-011**: System MUST support incremental updates by:
  - a) Computing content hash (SHA-256) of extracted text for each page
  - b) Storing content hash in Qdrant payload (`content_hash` field)
  - c) On re-run, comparing new content hash against stored hash
  - d) Only regenerating embeddings and upserting when hash differs
  - e) Optionally deleting vectors for pages that no longer exist (configurable, default: retain orphans)
- **FR-012**: System MUST normalize text (consistent whitespace, encoding) before embedding.
- **FR-013**: System MUST extract content from Docusaurus-specific HTML structure:
  - a) Primary selector: `article.markdown` or `.theme-doc-markdown`
  - b) Fallback selector: `main` element content
  - c) Strip elements: `nav`, `aside`, `.sidebar`, `.pagination-nav`, `.theme-doc-toc-*`, `.breadcrumbs`
  - d) Preserve elements: `pre` (code blocks), `table`, `blockquote`
  - e) Extract title from: `<h1>` or `<title>` or OpenGraph meta tags
  - f) Extract section headings from: `<h2>`, `<h3>` elements for chunk metadata

### Non-Functional Requirements

- **NFR-001**: System SHOULD process the entire book within a reasonable time frame (dependent on book size and API rate limits).
- **NFR-002**: System SHOULD minimize API calls through batching where supported.
- **NFR-003**: System MUST NOT store sensitive credentials in code (use environment variables).
- **NFR-004**: System SHOULD provide progress indication during execution.
- **NFR-005**: System MUST process pages at >= 10 pages/minute sustained throughput (excluding external API rate limits).
- **NFR-006**: System MUST complete embedding generation for a single chunk in < 500ms p95 (excluding network latency).
- **NFR-007**: System MUST consume < 500MB RAM during normal operation.
- **NFR-008**: Pipeline MUST complete for a 500-page book in < 60 minutes (with default rate limits).

### Key Entities

- **Document**: Represents a single Docusaurus page with URL, title, and raw HTML content.
- **Chunk**: A segment of extracted text with position metadata (index, character offsets) and parent document reference.
- **Vector**: The embedding representation of a chunk, stored with full metadata for retrieval traceability.
- **Run Report**: Summary of pipeline execution including counts, timing, and error log.

### Technical Configuration

#### Cohere Configuration
- **Model**: `embed-english-v3.0`
- **Embedding Dimensions**: 1024
- **Input Type**: `search_document` (for ingestion); `search_query` for retrieval (out of scope)
- **Batch Size**: Maximum 96 texts per API call
- **Truncation**: `END` (truncate end of text if exceeding model limit)

#### Qdrant Configuration
- **Collection Name**: Configurable via environment variable, default `rag_embedding`
- **Vector Configuration**: `{size: 1024, distance: "Cosine"}`
- **HNSW Index**: Use Qdrant defaults (m=16, ef_construct=100)
- **Payload Schema**:
  ```json
  {
    "source_url": "string (required)",
    "page_title": "string (required)",
    "chunk_index": "integer (required)",
    "section_heading": "string (optional)",
    "chunk_text": "string (required)",
    "content_hash": "string (required, SHA-256 for change detection)",
    "indexed_at": "ISO8601 datetime (required)"
  }
  ```

#### Chunking Configuration
- **Strategy**: Recursive character text splitter with semantic boundaries (paragraph > sentence)
- **Target Chunk Size**: 500 tokens (approximately 2000 characters)
- **Maximum Chunk Size**: 1000 tokens (hard limit)
- **Minimum Chunk Size**: 100 tokens (merge smaller chunks with adjacent)
- **Overlap**: 50 tokens between consecutive chunks

#### HTTP Client Configuration
- **Timeout**: 30 seconds per request
- **Max Concurrent Requests**: 5
- **Rate Limit**: 2 requests/second to source domain
- **User-Agent**: `DocusaurusIngestionPipeline/1.0`

#### Point ID Generation
- **Algorithm**: UUID derived from `SHA-256(source_url + "|" + chunk_index)` truncated to 32 hex characters
- **Deterministic**: Same URL + chunk index always produces same ID for idempotency

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All public book URLs (100%) are successfully fetched and processed, excluding any that return HTTP errors.
- **SC-002**: Each processed page produces at least one vector chunk stored in Qdrant (unless page has no text content).
- **SC-003**: Pipeline can be re-run without creating duplicate vectors (idempotency verified by consistent vector count across runs with unchanged content).
- **SC-004**: Every stored vector includes complete metadata: source URL, page title, chunk index, and section heading where applicable.
- **SC-005**: Pipeline generates a summary report showing: total URLs attempted, successful extractions, failures with reasons, and total vectors stored.
- **SC-006**: Developers can query Qdrant and retrieve relevant chunks with their source metadata for any topic covered in the book.

---

## Assumptions

- The Docusaurus book is deployed and publicly accessible via HTTPS.
- The book has a sitemap.xml or predictable URL structure for page discovery.
- Cohere API credentials and Qdrant connection details will be provided via environment variables.
- The book content is primarily text-based documentation (not a media-heavy site).
- Standard Cohere embedding model (e.g., embed-english-v3.0) will be used unless otherwise specified.
- Qdrant collection will be created if it doesn't exist, with appropriate vector dimensions matching Cohere model output.

---

## Out of Scope

- Real-time/streaming content updates (batch processing only)
- Query/retrieval API implementation (this spec covers ingestion only)
- User interface for pipeline management
- Multi-language embedding support (English only for initial implementation)
- Authentication-protected pages (public content only)
- Image/media content embedding

---

## Clarifications (Resolved)

The following design decisions were clarified during specification review:

| ID | Question | Decision | Rationale |
|----|----------|----------|-----------|
| Q1 | Docusaurus URL configuration | Environment variable (`DOCUSAURUS_BASE_URL`) | Flexibility to change URL without code changes; supports redeployments |
| Q2 | Qdrant collection lifecycle | Auto-create if missing | Self-bootstrapping pipeline, easier initial setup |
| Q3 | Stale content handling | Retain orphan vectors | Safe default, no accidental data loss; stale content can be manually cleaned |
| Q4 | Code block treatment | Embed as-is with surrounding text | Code searchable in context, natural chunking preserves meaning |
| Q5 | Qdrant hosting | Qdrant Cloud | Cloud-hosted with API key authentication |
| Q6 | Cohere API tier | Free tier | Conservative rate limiting (100 calls/min) to stay within limits |
| Q7 | Error threshold | Fail if > 5% pages fail | Balanced approach - tolerates transient errors but catches systemic issues |

---

## Environment Variables

The following environment variables MUST be configured before running the pipeline:

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `DOCUSAURUS_BASE_URL` | Yes | Base URL of deployed Docusaurus book | `https://my-book.vercel.app` |
| `COHERE_API_KEY` | Yes | Cohere API key for embeddings | `xxxxxxxxxxxxxxxx` |
| `QDRANT_URL` | Yes | Qdrant Cloud cluster URL | `https://xyz.cloud.qdrant.io:6333` |
| `QDRANT_API_KEY` | Yes | Qdrant Cloud API key | `xxxxxxxxxxxxxxxx` |
| `QDRANT_COLLECTION_NAME` | No | Collection name (default: `docusaurus_embeddings`) | `my_book_embeddings` |

---

## Dependencies

- **Cohere API**: For generating text embeddings
- **Qdrant Cloud/Instance**: For vector storage and retrieval
- **Deployed Docusaurus Book**: Source content to be indexed
- **Network Access**: To fetch book content and communicate with external APIs

---

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Cohere API rate limiting | Pipeline slowdown or failure | Implement batching, exponential backoff, and configurable delays |
| Large book causing memory issues | Pipeline crash | Process pages sequentially, stream content, don't load all at once |
| Qdrant downtime during ingestion | Partial data, inconsistent state | Implement checkpointing and resumable runs |
| HTML structure changes in Docusaurus | Text extraction fails | Use robust selectors, log warnings for unexpected structures |
