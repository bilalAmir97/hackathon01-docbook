# Quickstart: Embedding Pipeline

**Feature**: 004-Spec01-embedding-pipeline
**Date**: 2025-12-14

## Prerequisites

- Python 3.11+
- uv package manager (recommended) or pip
- Cohere API key (free tier available)
- Qdrant Cloud account and API key

## Setup

### 1. Create Backend Directory

```bash
cd hackathon-01
mkdir backend
cd backend
```

### 2. Initialize Project with uv

```bash
uv init
uv venv
source .venv/bin/activate  # Linux/macOS
# or
.venv\Scripts\activate     # Windows
```

### 3. Add Dependencies

```bash
uv add cohere qdrant-client httpx beautifulsoup4 lxml python-dotenv
```

Or with pip:

```bash
pip install cohere qdrant-client httpx beautifulsoup4 lxml python-dotenv
```

### 4. Create Environment File

Create `.env` in the backend directory:

```bash
# Required environment variables
DOCUSAURUS_BASE_URL=https://hackathon01-docbook-f1h8.vercel.app
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here
```

### 5. Create main.py

Create the single-file pipeline in `backend/main.py`:

```python
"""
Docusaurus to Qdrant Embedding Pipeline

Functions:
- get_all_urls(): Discover URLs from sitemap
- extract_text_from_url(): Fetch and extract text
- chunk_text(): Split text into chunks
- embed(): Generate Cohere embeddings
- create_collection(): Setup Qdrant collection
- save_chunk_to_qdrant(): Store vectors
- main(): Orchestrate pipeline
"""

import os
import hashlib
import json
from datetime import datetime
from dotenv import load_dotenv

# Implementation goes here...

if __name__ == "__main__":
    main()
```

## Running the Pipeline

### Basic Execution

```bash
cd backend
python main.py
```

### Expected Output

```json
{
  "run_id": "abc123...",
  "started_at": "2025-12-14T10:00:00Z",
  "completed_at": "2025-12-14T10:05:00Z",
  "duration_seconds": 300,
  "source_base_url": "https://hackathon01-docbook-f1h8.vercel.app",
  "statistics": {
    "urls_discovered": 47,
    "urls_attempted": 47,
    "urls_successful": 47,
    "urls_failed": 0,
    "chunks_created": 180,
    "vectors_upserted": 180,
    "vectors_unchanged": 0
  },
  "failures": [],
  "configuration": {
    "chunk_size": 2000,
    "chunk_overlap": 200,
    "cohere_model": "embed-english-v3.0",
    "qdrant_collection": "rag_embedding"
  }
}
```

## Verifying Results

### Check Qdrant Collection

```python
from qdrant_client import QdrantClient

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Get collection info
info = client.get_collection("rag_embedding")
print(f"Vectors count: {info.vectors_count}")

# Sample search
results = client.search(
    collection_name="rag_embedding",
    query_vector=[0.1] * 1024,  # Replace with actual query embedding
    limit=5
)
for r in results:
    print(f"- {r.payload['page_title']}: {r.score}")
```

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| Connection refused (Qdrant) | Check QDRANT_URL format includes port |
| 401 Unauthorized (Cohere) | Verify COHERE_API_KEY is valid |
| Empty sitemap | Verify DOCUSAURUS_BASE_URL is correct |
| Rate limit errors | Reduce batch size, increase delays |

### Debug Mode

Set environment variable for verbose logging:

```bash
export DEBUG=1
python main.py
```

## Re-running Pipeline

The pipeline is idempotent:
- Same URL + chunk index = same point ID
- Content hash detects unchanged pages
- Only modified content is re-embedded

```bash
# Safe to run multiple times
python main.py
python main.py  # No duplicates created
```

## Next Steps

After running the pipeline:

1. Verify vectors in Qdrant Cloud dashboard
2. Test semantic search queries
3. Build RAG retrieval API (out of scope for this spec)
