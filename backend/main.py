"""
Embedding Pipeline for Docusaurus Documentation

This module implements a document ingestion pipeline that:
1. Discovers URLs from a Docusaurus sitemap
2. Extracts text content from HTML pages
3. Chunks text with overlap for optimal embedding
4. Generates embeddings using Cohere's embed-english-v3.0 model
5. Stores vectors in Qdrant Cloud for RAG retrieval

Usage:
    python main.py

Environment Variables Required:
    DOCUSAURUS_BASE_URL: Base URL of the Docusaurus site
    COHERE_API_KEY: Cohere API key for embeddings
    QDRANT_URL: Qdrant Cloud cluster URL
    QDRANT_API_KEY: Qdrant API key

Author: Embedding Pipeline Team
Date: 2025-12-14
"""

import hashlib
import json
import os
import re
import time
import uuid
from datetime import datetime, timezone
from typing import Any
from urllib.parse import urlparse
from xml.etree import ElementTree

import cohere
import httpx
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models as qdrant_models

# Load environment variables
load_dotenv()

# Configuration constants
CHUNK_SIZE = 2000  # ~500 tokens
CHUNK_OVERLAP = 200  # ~50 tokens (10% overlap)
COHERE_MODEL = "embed-english-v3.0"
COHERE_DIMENSIONS = 1024
QDRANT_COLLECTION = "rag_embedding"
COHERE_BATCH_SIZE = 96
HTTP_TIMEOUT = 30
RATE_LIMIT_DELAY = 0.5  # 2 requests per second


def compute_content_hash(text: str) -> str:
    """
    Compute SHA-256 hash of normalized text content.

    Used for change detection to enable idempotent re-runs.
    Text is normalized by stripping whitespace and lowercasing.

    Args:
        text: The text content to hash.

    Returns:
        A 64-character hexadecimal SHA-256 hash string.

    Example:
        >>> compute_content_hash("Hello World")
        'a591a6d40bf420404a011733cfb7b190d62c65bf0bcda32b57b277d9ad9f146e'
    """
    normalized = text.strip().lower()
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def compute_point_id(source_url: str, chunk_index: int) -> str:
    """
    Compute deterministic point ID for Qdrant.

    Uses SHA-256 hash of URL + chunk_index for idempotent upserts.
    The same content will always produce the same ID.

    Args:
        source_url: Full URL of the source page.
        chunk_index: 0-based index of the chunk within the page.

    Returns:
        A 32-character hexadecimal string suitable for Qdrant point ID.

    Example:
        >>> compute_point_id("https://example.com/page", 0)
        'a1b2c3d4e5f6...'
    """
    combined = f"{source_url}|{chunk_index}"
    full_hash = hashlib.sha256(combined.encode("utf-8")).hexdigest()
    return full_hash[:32]


def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list[dict]:
    """
    Split text into overlapping chunks with metadata.

    Uses character-based splitting with paragraph/sentence boundary awareness.
    Each chunk includes position metadata for reconstruction.

    Args:
        text: The full text content to chunk.
        chunk_size: Maximum characters per chunk (default: 2000).
        overlap: Number of overlapping characters between chunks (default: 200).

    Returns:
        List of chunk dictionaries with keys:
            - text: The chunk text content
            - char_start: Starting character position
            - char_end: Ending character position
            - chunk_index: 0-based chunk index

    Example:
        >>> chunks = chunk_text("Long text...", chunk_size=100, overlap=20)
        >>> chunks[0]
        {'text': '...', 'char_start': 0, 'char_end': 100, 'chunk_index': 0}
    """
    if not text or len(text.strip()) < 50:
        return []

    text = text.strip()
    chunks = []
    start = 0
    chunk_index = 0

    while start < len(text):
        end = start + chunk_size

        if end < len(text):
            # Try to break at paragraph boundary
            paragraph_break = text.rfind("\n\n", start, end)
            if paragraph_break > start + chunk_size // 2:
                end = paragraph_break + 2
            else:
                # Try to break at sentence boundary
                sentence_break = max(
                    text.rfind(". ", start, end),
                    text.rfind("! ", start, end),
                    text.rfind("? ", start, end),
                )
                if sentence_break > start + chunk_size // 2:
                    end = sentence_break + 2
        else:
            end = len(text)

        chunk_text_content = text[start:end].strip()

        if chunk_text_content:
            chunks.append({
                "text": chunk_text_content,
                "char_start": start,
                "char_end": end,
                "chunk_index": chunk_index,
            })
            chunk_index += 1

        # Move start position with overlap
        start = end - overlap if end < len(text) else len(text)

    return chunks


def get_all_urls(base_url: str) -> list[str]:
    """
    Discover all documentation URLs from sitemap.xml.

    Parses the Docusaurus sitemap to extract all page URLs.
    Filters to include only /docs/ paths.
    Rewrites placeholder domains to the actual base URL.

    Args:
        base_url: Base URL of the Docusaurus site.

    Returns:
        List of full URLs found in the sitemap.

    Raises:
        httpx.HTTPError: If sitemap fetch fails.

    Example:
        >>> urls = get_all_urls("https://example.com")
        >>> len(urls)
        47
    """
    sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"

    with httpx.Client(timeout=HTTP_TIMEOUT) as client:
        response = client.get(sitemap_url)
        response.raise_for_status()

    # Parse sitemap XML
    root = ElementTree.fromstring(response.content)

    # Handle XML namespace
    namespace = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"}

    urls = []
    base_url_clean = base_url.rstrip("/")

    for url_elem in root.findall(".//ns:url/ns:loc", namespace):
        url = url_elem.text
        if url and "/docs/" in url:
            # Extract path from URL and rewrite with correct base URL
            # This handles cases where sitemap has placeholder domains
            parsed = urlparse(url)
            path = parsed.path
            # Reconstruct URL with the actual base URL
            corrected_url = f"{base_url_clean}{path}"
            urls.append(corrected_url)

    return urls


def extract_text_from_url(url: str) -> dict[str, Any]:
    """
    Fetch URL and extract clean text content.

    Uses Docusaurus-specific selectors to extract main content.
    Strips navigation, sidebar, and other non-content elements.

    Args:
        url: Full URL of the page to extract.

    Returns:
        Dictionary with keys:
            - url: Source URL
            - title: Page title
            - text_content: Extracted clean text
            - section_headings: List of H2/H3 headings
            - content_hash: SHA-256 of text content

    Raises:
        httpx.HTTPError: If page fetch fails.

    Example:
        >>> result = extract_text_from_url("https://example.com/docs/intro")
        >>> result['title']
        'Introduction'
    """
    with httpx.Client(timeout=HTTP_TIMEOUT) as client:
        response = client.get(url)
        response.raise_for_status()

    soup = BeautifulSoup(response.content, "lxml")

    # Extract title
    title = ""
    title_elem = soup.find("h1")
    if title_elem:
        title = title_elem.get_text(strip=True)
    elif soup.title:
        title = soup.title.get_text(strip=True)

    # Find main content using Docusaurus selectors
    content_elem = (
        soup.find("article", class_="markdown") or
        soup.find("div", class_="theme-doc-markdown") or
        soup.find("main") or
        soup.find("article")
    )

    if not content_elem:
        content_elem = soup.body if soup.body else soup

    # Remove non-content elements
    for selector in ["nav", "aside", ".sidebar", ".pagination-nav",
                     ".breadcrumbs", ".table-of-contents", "footer",
                     ".theme-doc-toc-mobile", ".theme-doc-sidebar-container"]:
        for elem in content_elem.select(selector):
            elem.decompose()

    # Extract section headings
    section_headings = []
    for heading in content_elem.find_all(["h2", "h3"]):
        heading_text = heading.get_text(strip=True)
        if heading_text:
            section_headings.append(heading_text)

    # Extract text content
    text_content = content_elem.get_text(separator="\n", strip=True)

    # Clean up whitespace
    text_content = re.sub(r"\n{3,}", "\n\n", text_content)
    text_content = re.sub(r" {2,}", " ", text_content)

    content_hash = compute_content_hash(text_content)

    return {
        "url": url,
        "title": title,
        "text_content": text_content,
        "section_headings": section_headings,
        "content_hash": content_hash,
    }


def create_collection(client: QdrantClient) -> bool:
    """
    Create or verify Qdrant collection exists.

    Creates the rag_embedding collection with proper vector configuration
    if it doesn't exist. Uses Cosine distance for similarity.

    Args:
        client: Initialized QdrantClient instance.

    Returns:
        True if collection was created, False if it already exists.

    Example:
        >>> client = QdrantClient(url="...", api_key="...")
        >>> created = create_collection(client)
        >>> print("Created new collection" if created else "Collection exists")
    """
    collections = client.get_collections().collections
    collection_names = [c.name for c in collections]

    if QDRANT_COLLECTION in collection_names:
        return False

    client.create_collection(
        collection_name=QDRANT_COLLECTION,
        vectors_config=qdrant_models.VectorParams(
            size=COHERE_DIMENSIONS,
            distance=qdrant_models.Distance.COSINE,
        ),
        hnsw_config=qdrant_models.HnswConfigDiff(
            m=16,
            ef_construct=100,
        ),
        on_disk_payload=True,
    )

    # Create payload indexes for filtering
    client.create_payload_index(
        collection_name=QDRANT_COLLECTION,
        field_name="source_url",
        field_schema=qdrant_models.PayloadSchemaType.KEYWORD,
    )
    client.create_payload_index(
        collection_name=QDRANT_COLLECTION,
        field_name="content_hash",
        field_schema=qdrant_models.PayloadSchemaType.KEYWORD,
    )

    return True


def embed(texts: list[str], cohere_client: cohere.Client) -> list[list[float]]:
    """
    Generate embeddings for a batch of texts using Cohere.

    Uses embed-english-v3.0 model with search_document input type.
    Implements exponential backoff for rate limit handling.

    Args:
        texts: List of text strings to embed.
        cohere_client: Initialized Cohere client.

    Returns:
        List of embedding vectors (each 1024 floats).

    Raises:
        cohere.CohereAPIError: If API call fails after retries.

    Example:
        >>> client = cohere.Client(api_key="...")
        >>> embeddings = embed(["Hello world"], client)
        >>> len(embeddings[0])
        1024
    """
    max_retries = 3
    base_delay = 1

    for attempt in range(max_retries):
        try:
            response = cohere_client.embed(
                texts=texts,
                model=COHERE_MODEL,
                input_type="search_document",
            )
            return response.embeddings
        except Exception as e:
            if attempt < max_retries - 1:
                delay = base_delay * (2 ** attempt)
                print(f"  Cohere API error, retrying in {delay}s: {e}")
                time.sleep(delay)
            else:
                raise


def save_chunk_to_qdrant(
    client: QdrantClient,
    point_id: str,
    vector: list[float],
    payload: dict[str, Any],
) -> None:
    """
    Upsert a single chunk vector to Qdrant.

    Uses deterministic point ID for idempotent updates.

    Args:
        client: Initialized QdrantClient instance.
        point_id: 32-character hexadecimal point ID.
        vector: 1024-dimensional embedding vector.
        payload: Metadata payload dictionary.

    Example:
        >>> save_chunk_to_qdrant(client, "abc123...", [0.1, 0.2, ...], {...})
    """
    client.upsert(
        collection_name=QDRANT_COLLECTION,
        points=[
            qdrant_models.PointStruct(
                id=point_id,
                vector=vector,
                payload=payload,
            )
        ],
    )


def check_content_changed(client: QdrantClient, point_id: str, content_hash: str) -> bool:
    """
    Check if content has changed since last indexing.

    Compares the content hash of a chunk against the stored hash.
    Used for idempotent re-runs to skip unchanged content.

    Args:
        client: Initialized QdrantClient instance.
        point_id: 32-character hexadecimal point ID.
        content_hash: SHA-256 hash of current content.

    Returns:
        True if content has changed or doesn't exist, False if unchanged.

    Example:
        >>> changed = check_content_changed(client, "abc123...", "hash...")
        >>> if changed:
        ...     # Re-embed and upsert
    """
    try:
        points = client.retrieve(
            collection_name=QDRANT_COLLECTION,
            ids=[point_id],
            with_payload=True,
        )

        if not points:
            return True

        stored_hash = points[0].payload.get("content_hash", "")
        return stored_hash != content_hash

    except Exception:
        return True


def main() -> None:
    """
    Main pipeline orchestration function.

    Executes the full embedding pipeline:
    1. Validates environment configuration
    2. Discovers URLs from sitemap
    3. Extracts text from each page
    4. Chunks and embeds content
    5. Stores vectors in Qdrant
    6. Generates run report
    """
    # Track timing
    started_at = datetime.now(timezone.utc)
    run_id = str(uuid.uuid4())

    print(f"Starting embedding pipeline run: {run_id}")
    print(f"Started at: {started_at.isoformat()}")

    # Validate environment variables
    required_vars = ["DOCUSAURUS_BASE_URL", "COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"ERROR: Missing environment variables: {', '.join(missing_vars)}")
        return

    base_url = os.getenv("DOCUSAURUS_BASE_URL")

    # Initialize clients
    print("\nInitializing clients...")
    cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        timeout=30,
    )

    # Create collection if needed
    collection_created = create_collection(qdrant_client)
    if collection_created:
        print(f"Created new collection: {QDRANT_COLLECTION}")
    else:
        print(f"Using existing collection: {QDRANT_COLLECTION}")

    # Statistics tracking
    stats = {
        "urls_discovered": 0,
        "urls_attempted": 0,
        "urls_successful": 0,
        "urls_failed": 0,
        "chunks_created": 0,
        "vectors_upserted": 0,
        "vectors_unchanged": 0,
    }
    failures = []

    # Discover URLs
    print(f"\nDiscovering URLs from {base_url}...")
    try:
        urls = get_all_urls(base_url)
        stats["urls_discovered"] = len(urls)
        print(f"Found {len(urls)} URLs")
    except Exception as e:
        print(f"ERROR: Failed to fetch sitemap: {e}")
        return

    # Process each URL
    print("\nProcessing pages...")
    for i, url in enumerate(urls, 1):
        stats["urls_attempted"] += 1
        print(f"\n[{i}/{len(urls)}] {url}")

        try:
            # Rate limiting
            time.sleep(RATE_LIMIT_DELAY)

            # Extract text
            doc = extract_text_from_url(url)
            print(f"  Title: {doc['title']}")
            print(f"  Text length: {len(doc['text_content'])} chars")

            if len(doc["text_content"]) < 50:
                print("  Skipping: insufficient content")
                continue

            # Chunk text
            chunks = chunk_text(doc["text_content"])
            print(f"  Chunks: {len(chunks)}")
            stats["chunks_created"] += len(chunks)

            if not chunks:
                print("  Skipping: no chunks created")
                continue

            # Process chunks in batches
            batch_texts = []
            batch_metadata = []

            for chunk in chunks:
                point_id = compute_point_id(url, chunk["chunk_index"])
                chunk_hash = compute_content_hash(chunk["text"])

                # Check if content changed
                if not check_content_changed(qdrant_client, point_id, chunk_hash):
                    stats["vectors_unchanged"] += 1
                    continue

                batch_texts.append(chunk["text"])

                # Find nearest section heading
                section_heading = None
                for heading in doc["section_headings"]:
                    if heading.lower() in chunk["text"].lower():
                        section_heading = heading
                        break

                batch_metadata.append({
                    "point_id": point_id,
                    "payload": {
                        "source_url": url,
                        "page_title": doc["title"],
                        "chunk_index": chunk["chunk_index"],
                        "chunk_text": chunk["text"],
                        "section_heading": section_heading,
                        "content_hash": chunk_hash,
                        "indexed_at": datetime.now(timezone.utc).isoformat(),
                    },
                })

                # Process batch when full
                if len(batch_texts) >= COHERE_BATCH_SIZE:
                    embeddings = embed(batch_texts, cohere_client)
                    for j, emb in enumerate(embeddings):
                        save_chunk_to_qdrant(
                            qdrant_client,
                            batch_metadata[j]["point_id"],
                            emb,
                            batch_metadata[j]["payload"],
                        )
                        stats["vectors_upserted"] += 1
                    batch_texts = []
                    batch_metadata = []

            # Process remaining batch
            if batch_texts:
                embeddings = embed(batch_texts, cohere_client)
                for j, emb in enumerate(embeddings):
                    save_chunk_to_qdrant(
                        qdrant_client,
                        batch_metadata[j]["point_id"],
                        emb,
                        batch_metadata[j]["payload"],
                    )
                    stats["vectors_upserted"] += 1

            stats["urls_successful"] += 1

        except Exception as e:
            stats["urls_failed"] += 1
            failures.append({
                "url": url,
                "error_code": type(e).__name__,
                "error_message": str(e),
            })
            print(f"  ERROR: {e}")

    # Generate run report
    completed_at = datetime.now(timezone.utc)
    duration = (completed_at - started_at).total_seconds()

    run_report = {
        "run_id": run_id,
        "started_at": started_at.isoformat(),
        "completed_at": completed_at.isoformat(),
        "duration_seconds": round(duration, 2),
        "source_base_url": base_url,
        "statistics": stats,
        "failures": failures,
        "configuration": {
            "chunk_size": CHUNK_SIZE,
            "chunk_overlap": CHUNK_OVERLAP,
            "cohere_model": COHERE_MODEL,
            "qdrant_collection": QDRANT_COLLECTION,
        },
    }

    # Check error threshold (5%)
    if stats["urls_attempted"] > 0:
        error_rate = stats["urls_failed"] / stats["urls_attempted"]
        if error_rate > 0.05:
            print(f"\nWARNING: Error rate ({error_rate:.1%}) exceeds 5% threshold")

    # Print summary
    print("\n" + "=" * 60)
    print("PIPELINE RUN COMPLETE")
    print("=" * 60)
    print(f"Run ID: {run_id}")
    print(f"Duration: {duration:.2f} seconds")
    print(f"URLs: {stats['urls_successful']}/{stats['urls_attempted']} successful")
    print(f"Chunks created: {stats['chunks_created']}")
    print(f"Vectors upserted: {stats['vectors_upserted']}")
    print(f"Vectors unchanged: {stats['vectors_unchanged']}")

    if failures:
        print(f"\nFailures ({len(failures)}):")
        for f in failures[:5]:  # Show first 5
            print(f"  - {f['url']}: {f['error_message']}")

    # Output JSON report
    print("\n--- JSON Run Report ---")
    print(json.dumps(run_report, indent=2))


if __name__ == "__main__":
    main()
