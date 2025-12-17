# Implementation Plan: Embedding Pipeline Setup

**Branch**: `004-Spec01-embedding-pipeline` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-Spec01-embedding-pipeline/spec.md`

**Note**: This template is filled in by the `/sp.plan` command.

## Summary

Build a document ingestion pipeline that extracts text from a deployed Docusaurus book (47 URLs across 2 modules), generates embeddings using Cohere's `embed-english-v3.0` model, and stores vectors in Qdrant Cloud for RAG retrieval. The pipeline will be implemented as a single `main.py` file with functions for URL discovery, text extraction, chunking, embedding, and vector storage. Idempotency is ensured via deterministic point IDs and content hashing.

## Technical Context

**Language/Version**: Python 3.11 (managed via `uv`)
**Primary Dependencies**: `cohere` (embeddings), `qdrant-client` (vector DB), `httpx` (async HTTP), `beautifulsoup4` + `lxml` (HTML parsing), `python-dotenv` (env vars)
**Storage**: Qdrant Cloud (collection: `rag_embedding`)
**Testing**: pytest (unit tests for chunking, ID generation, text extraction)
**Target Platform**: CLI tool (cross-platform: Windows/Linux/macOS)
**Project Type**: Single-file CLI application (`main.py`)
**Performance Goals**: >=10 pages/minute throughput, <500ms p95 per embedding call
**Constraints**: <500MB RAM, respect Cohere free tier (100 calls/min), 2 req/s to source domain
**Scale/Scope**: ~47 URLs, ~100-200 chunks estimated, single collection

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Embodied Intelligence | N/A | This feature is infrastructure for RAG, not robotics content |
| II. Simulation-First | N/A | Backend pipeline, no simulation involved |
| III. Technical Rigor | PASS | Python code will be syntactically correct and reproducible |
| IV. Clarity & Structure | PASS | Single-file design per user request, clear function organization |
| V. Accessibility & Readability | PASS | Code will include docstrings and comments |
| VI. Verification & Citation | PASS | Uses official Cohere/Qdrant APIs, no hallucinated endpoints |

**Gate Result**: PASS - No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/004-Spec01-embedding-pipeline/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Single-file pipeline with all functions
├── pyproject.toml       # uv project configuration
├── .env.example         # Environment variable template
└── .python-version      # Python version pinning (3.11)
```

**Structure Decision**: Single-file design as explicitly requested by user. All pipeline logic consolidated in `main.py` with the following functions:
- `get_all_urls()` - URL discovery from sitemap
- `extract_text_from_url()` - HTML fetch + text extraction
- `chunk_text()` - Text chunking with overlap
- `embed()` - Cohere embedding generation
- `create_collection()` - Qdrant collection setup (named `rag_embedding`)
- `save_chunk_to_qdrant()` - Vector upsert with metadata
- `main()` - Orchestration and run report

## Complexity Tracking

> No constitution violations - this section is not applicable.

## Architecture Overview

### Data Flow

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  get_all_urls() │────>│ extract_text_   │────>│   chunk_text()  │
│  (sitemap.xml)  │     │ from_url()      │     │  (500 tokens)   │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                                        │
                                                        v
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│     main()      │<────│ save_chunk_to_  │<────│     embed()     │
│  (run report)   │     │ qdrant()        │     │ (Cohere batch)  │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                              │
                              v
                        ┌─────────────────┐
                        │create_collection│
                        │ (rag_embedding) │
                        └─────────────────┘
```

### Key Design Decisions

1. **Single File Architecture**: All code in `main.py` per user requirement
2. **Sitemap-First Discovery**: Parse `sitemap.xml` for URLs (47 URLs available)
3. **Deterministic Point IDs**: SHA-256 hash of `source_url + "|" + chunk_index`
4. **Content Hashing**: SHA-256 of extracted text for idempotent re-runs
5. **Batch Embedding**: Up to 96 texts per Cohere API call
6. **Collection Name**: `rag_embedding` (as specified by user)

### Error Handling Strategy

- HTTP errors: Log and skip, continue with remaining URLs
- Cohere rate limits: Exponential backoff with max 3 retries
- Qdrant errors: Retry with backoff, fail gracefully after max retries
- Empty pages: Log warning, skip vector creation

### Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `DOCUSAURUS_BASE_URL` | Yes | - | `https://hackathon01-docbook-f1h8.vercel.app` |
| `COHERE_API_KEY` | Yes | - | Cohere API key |
| `QDRANT_URL` | Yes | - | Qdrant Cloud URL |
| `QDRANT_API_KEY` | Yes | - | Qdrant API key |

## Phase Outputs Reference

- **Phase 0**: `research.md` - Design decisions and rationale
- **Phase 1**: `data-model.md`, `contracts/`, `quickstart.md`
- **Phase 2**: `tasks.md` (created by `/sp.tasks`, not `/sp.plan`)
