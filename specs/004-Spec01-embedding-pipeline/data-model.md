# Data Model: Embedding Pipeline

**Feature**: 004-Spec01-embedding-pipeline
**Date**: 2025-12-14
**Status**: Complete

## Overview

This document defines the data entities, their relationships, and the Qdrant payload schema for the embedding pipeline.

---

## Core Entities

### 1. Document

Represents a single Docusaurus page fetched from the source site.

| Field | Type | Description |
|-------|------|-------------|
| url | str | Full URL of the page |
| title | str | Page title (from h1 or meta) |
| html_content | str | Raw HTML content |
| text_content | str | Extracted clean text |
| content_hash | str | SHA-256 of text_content |
| fetched_at | datetime | Timestamp of fetch |
| section_headings | list[str] | H2/H3 headings found in page |

**Validation Rules**:
- url must be a valid HTTPS URL
- title must be non-empty (fallback to URL slug if missing)
- content_hash is computed after text normalization

---

### 2. Chunk

A segment of extracted text with position metadata.

| Field | Type | Description |
|-------|------|-------------|
| document_url | str | Parent document URL |
| chunk_index | int | 0-based position in document |
| text | str | Chunk text content |
| char_start | int | Character offset start |
| char_end | int | Character offset end |
| section_heading | str or None | Nearest H2/H3 heading |
| point_id | str | Deterministic ID for Qdrant |

**Point ID Generation**:
- Algorithm: SHA-256(source_url + "|" + chunk_index)[:32]
- Ensures idempotent upserts

**Validation Rules**:
- chunk_index must be >= 0
- text length must be between 100 and 2000 characters (approximately)
- char_start < char_end

---

### 3. Vector (Qdrant Point)

The embedding representation stored in Qdrant.

| Field | Type | Description |
|-------|------|-------------|
| id | str | Deterministic point ID (32 hex chars) |
| vector | list[float] | 1024-dimensional embedding |
| payload | dict | Metadata payload |

---

### 4. RunReport

Summary of pipeline execution.

| Field | Type | Description |
|-------|------|-------------|
| run_id | str | UUID for this run |
| started_at | datetime | Start timestamp |
| completed_at | datetime | End timestamp |
| duration_seconds | float | Total duration |
| source_base_url | str | Base URL processed |
| statistics | dict | Processing statistics |
| failures | list | Failed URL records |
| configuration | dict | Run configuration |

---

## Qdrant Payload Schema

Each vector point in Qdrant includes the following payload:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| source_url | string | Yes | Full URL of source page |
| page_title | string | Yes | Document title |
| chunk_index | integer | Yes | 0-based chunk position |
| chunk_text | string | Yes | Full text of chunk (for display) |
| section_heading | string | No | Nearest heading (null if none) |
| content_hash | string | Yes | SHA-256 for change detection |
| indexed_at | string | Yes | ISO8601 timestamp |

### Example Payload

```json
{
  "source_url": "https://hackathon01-docbook-f1h8.vercel.app/docs/module-01/chapter-01/lesson-01",
  "page_title": "Introduction to ROS 2",
  "chunk_index": 0,
  "chunk_text": "ROS 2 (Robot Operating System 2) is the next generation...",
  "section_heading": "What is ROS 2?",
  "content_hash": "a3f2b8c9d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1",
  "indexed_at": "2025-12-14T10:30:00Z"
}
```

---

## Entity Relationships

```
Sitemap (47 URLs)
    │
    │ 1:N
    ▼
Document
    │ - url (PK)
    │ - title
    │ - text_content
    │ - content_hash
    │
    │ 1:N (typically 1-10 chunks/doc)
    ▼
Chunk
    │ - document_url (FK)
    │ - chunk_index
    │ - text
    │ - point_id (computed)
    │
    │ 1:1
    ▼
VectorPoint (Qdrant)
    - id (PK) = chunk.point_id
    - vector[1024]
    - payload
```

---

## Collection Configuration

### Qdrant Collection: rag_embedding

| Setting | Value | Description |
|---------|-------|-------------|
| name | rag_embedding | Collection name |
| vector size | 1024 | Matches Cohere embed-english-v3.0 |
| distance | Cosine | Similarity metric |
| HNSW m | 16 | Index parameter |
| HNSW ef_construct | 100 | Index parameter |
| on_disk_payload | True | Store payloads on disk |

### Recommended Payload Indexes

| Field | Index Type | Purpose |
|-------|------------|---------|
| source_url | keyword | Filter by page |
| page_title | keyword | Filter by title |
| content_hash | keyword | Change detection |

---

## Run Report JSON Schema

```json
{
  "run_id": "UUID",
  "started_at": "ISO8601",
  "completed_at": "ISO8601",
  "duration_seconds": 123.45,
  "source_base_url": "https://hackathon01-docbook-f1h8.vercel.app",
  "statistics": {
    "urls_discovered": 47,
    "urls_attempted": 47,
    "urls_successful": 45,
    "urls_failed": 2,
    "chunks_created": 180,
    "vectors_upserted": 180,
    "vectors_unchanged": 0
  },
  "failures": [
    {"url": "...", "error_code": "HTTP_404", "error_message": "Page not found"}
  ],
  "configuration": {
    "chunk_size": 500,
    "chunk_overlap": 50,
    "cohere_model": "embed-english-v3.0",
    "qdrant_collection": "rag_embedding"
  }
}
```
