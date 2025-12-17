"""Core retrieval interface for semantic similarity search.

This module provides the main retrieve() function for querying the Qdrant
vector database using Cohere embeddings. It supports metadata filtering,
deterministic result ordering, and retry with exponential backoff.
"""

from __future__ import annotations

import os
import time
from typing import Any

import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models as qdrant_models

from config import (
    COHERE_INPUT_TYPE_QUERY,
    COHERE_MODEL,
    FILTERABLE_FIELDS,
    MAX_RETRIES_CONNECTION,
    MAX_RETRIES_EMBEDDING,
    QDRANT_COLLECTION,
)
from errors import (
    CollectionNotFoundError,
    EmbeddingError,
    InvalidFilterError,
    QdrantConnectionError,
    QdrantTimeoutError,
    retry_with_backoff,
)
from models import ChunkResult, RetrievalRequest, SearchResult

# Load environment variables
load_dotenv()


# ============================================================================
# Client Initialization
# ============================================================================


def get_cohere_client() -> cohere.Client:
    """Get Cohere client with API key from environment."""
    api_key = os.environ.get("COHERE_API_KEY")
    if not api_key:
        from errors import ConfigurationError
        raise ConfigurationError("COHERE_API_KEY environment variable not set")
    return cohere.Client(api_key)


def get_qdrant_client() -> QdrantClient:
    """Get Qdrant client with credentials from environment."""
    url = os.environ.get("QDRANT_URL")
    api_key = os.environ.get("QDRANT_API_KEY")
    if not url:
        from errors import ConfigurationError
        raise ConfigurationError("QDRANT_URL environment variable not set")
    if not api_key:
        from errors import ConfigurationError
        raise ConfigurationError("QDRANT_API_KEY environment variable not set")
    return QdrantClient(url=url, api_key=api_key)


# ============================================================================
# Embedding Function
# ============================================================================


def embed_query(query: str, client: cohere.Client | None = None) -> list[float]:
    """Generate embedding for a query string using Cohere.

    Args:
        query: Natural language query to embed
        client: Optional Cohere client (creates new one if not provided)

    Returns:
        1024-dimensional embedding vector

    Raises:
        EmbeddingError: If Cohere API fails
    """
    if client is None:
        client = get_cohere_client()

    try:
        response = client.embed(
            texts=[query],
            model=COHERE_MODEL,
            input_type=COHERE_INPUT_TYPE_QUERY,
        )
        return response.embeddings[0]
    except Exception as e:
        raise EmbeddingError(
            message=f"Failed to generate embedding: {str(e)}",
            details={"query_length": len(query), "error": str(e)},
        )


# ============================================================================
# Filter Building
# ============================================================================


def build_filter(filters: dict[str, str | list[str]] | None) -> qdrant_models.Filter | None:
    """Build Qdrant filter from metadata filter dictionary.

    Args:
        filters: Dictionary of field -> value(s) filters
            - Single value: exact match (FieldCondition with MatchValue)
            - List of values: OR match (FieldCondition with MatchAny)
            - Multiple fields: AND composition

    Returns:
        Qdrant Filter object or None if no filters

    Raises:
        InvalidFilterError: If filter field is not in FILTERABLE_FIELDS
    """
    if not filters:
        return None

    conditions = []

    for field_name, value in filters.items():
        # Validate field name
        if field_name not in FILTERABLE_FIELDS:
            raise InvalidFilterError(field_name, FILTERABLE_FIELDS)

        # Build condition based on value type
        if isinstance(value, list):
            # Multiple values -> OR (MatchAny)
            condition = qdrant_models.FieldCondition(
                key=field_name,
                match=qdrant_models.MatchAny(any=value),
            )
        else:
            # Single value -> exact match
            condition = qdrant_models.FieldCondition(
                key=field_name,
                match=qdrant_models.MatchValue(value=value),
            )

        conditions.append(condition)

    # Combine with AND logic
    if len(conditions) == 1:
        return qdrant_models.Filter(must=[conditions[0]])
    else:
        return qdrant_models.Filter(must=conditions)


# ============================================================================
# Core Retrieval
# ============================================================================


def retrieve(
    request: RetrievalRequest,
    cohere_client: cohere.Client | None = None,
    qdrant_client: QdrantClient | None = None,
) -> SearchResult:
    """Retrieve semantically similar chunks from Qdrant.

    Args:
        request: RetrievalRequest with query, top_k, and optional filters
        cohere_client: Optional Cohere client
        qdrant_client: Optional Qdrant client

    Returns:
        SearchResult with ranked ChunkResult objects and timing

    Raises:
        InvalidQueryError: If query validation fails
        EmbeddingError: If Cohere API fails
        QdrantConnectionError: If Qdrant is unreachable
        QdrantTimeoutError: If search times out
        InvalidFilterError: If filter field is invalid
    """
    total_start = time.perf_counter()

    # Initialize clients if not provided
    if cohere_client is None:
        cohere_client = get_cohere_client()
    if qdrant_client is None:
        qdrant_client = get_qdrant_client()

    # Step 1: Generate query embedding
    embed_start = time.perf_counter()
    query_vector = embed_query(request.query, cohere_client)
    embedding_ms = (time.perf_counter() - embed_start) * 1000

    # Step 2: Build filter
    query_filter = build_filter(request.filters)

    # Step 3: Search Qdrant using query_points (qdrant-client >= 1.7)
    search_start = time.perf_counter()
    try:
        response = qdrant_client.query_points(
            collection_name=QDRANT_COLLECTION,
            query=query_vector,
            limit=request.top_k,
            query_filter=query_filter,
            with_payload=True,
            score_threshold=request.score_threshold,
        )
        search_results = response.points
    except Exception as e:
        error_str = str(e).lower()
        if "timeout" in error_str:
            raise QdrantTimeoutError(timeout_ms=3000, details={"error": str(e)})
        elif "connection" in error_str or "unreachable" in error_str:
            raise QdrantConnectionError(message=f"Qdrant connection failed: {str(e)}")
        elif "not found" in error_str or "doesn't exist" in error_str:
            raise CollectionNotFoundError(QDRANT_COLLECTION)
        else:
            raise QdrantConnectionError(message=f"Qdrant search failed: {str(e)}")

    search_ms = (time.perf_counter() - search_start) * 1000

    # Step 4: Map results to ChunkResult objects
    chunk_results = []
    for point in search_results:
        payload = point.payload or {}
        chunk = ChunkResult(
            id=str(point.id),
            score=point.score,
            chunk_text=payload.get("chunk_text", ""),
            source_url=payload.get("source_url", ""),
            page_title=payload.get("page_title", ""),
            section_heading=payload.get("section_heading"),
            chunk_index=payload.get("chunk_index", 0),
        )
        chunk_results.append(chunk)

    # Step 5: Apply deterministic sorting (score DESC, id ASC for ties)
    chunk_results.sort(key=lambda x: (-x.score, x.id))

    # Calculate total latency
    total_ms = (time.perf_counter() - total_start) * 1000

    return SearchResult(
        query=request.query,
        results=chunk_results,
        total_found=len(chunk_results),
        latency_ms=total_ms,
        embedding_ms=embedding_ms,
        search_ms=search_ms,
    )


# ============================================================================
# Retry Wrapper
# ============================================================================


def retrieve_with_retry(
    request: RetrievalRequest,
    cohere_client: cohere.Client | None = None,
    qdrant_client: QdrantClient | None = None,
    max_retries: int = MAX_RETRIES_CONNECTION,
) -> SearchResult:
    """Retrieve with automatic retry on transient errors.

    Args:
        request: RetrievalRequest
        cohere_client: Optional Cohere client
        qdrant_client: Optional Qdrant client
        max_retries: Maximum retry attempts

    Returns:
        SearchResult from successful retrieval

    Raises:
        Same exceptions as retrieve() after retries exhausted
    """
    def do_retrieve() -> SearchResult:
        return retrieve(request, cohere_client, qdrant_client)

    return retry_with_backoff(
        func=do_retrieve,
        max_retries=max_retries,
    )


# ============================================================================
# CLI Entry Point
# ============================================================================


def main():
    """CLI entry point for retrieval operations."""
    import argparse
    import json
    from dataclasses import asdict

    parser = argparse.ArgumentParser(description="RAG Retrieval Pipeline")
    parser.add_argument("--query", "-q", type=str, help="Query string for retrieval")
    parser.add_argument("--top-k", "-k", type=int, default=5, help="Number of results")
    parser.add_argument("--validate", action="store_true", help="Run validation checks")
    parser.add_argument("--evaluate", action="store_true", help="Run evaluation with golden queries")
    parser.add_argument("--json", action="store_true", help="Output as JSON")

    args = parser.parse_args()

    if args.validate:
        from validation import validate
        report = validate()
        if args.json:
            print(json.dumps(asdict(report), default=str, indent=2))
        else:
            print(f"Validation: {report.overall_status.upper()}")
            for name, result in report.checks.items():
                print(f"  {name}: {result.status} - {result.message}")
        return 0 if report.overall_status == "pass" else 1

    if args.evaluate:
        from evaluation import evaluate
        report = evaluate()
        if args.json:
            print(json.dumps(asdict(report), default=str, indent=2))
        else:
            print(f"Evaluation: {report.overall_status.upper()}")
            print(f"  MRR: {report.summary.mrr_average:.3f}")
            print(f"  Hit@1: {report.summary.hit_at_1_rate:.1%}")
            print(f"  Hit@5: {report.summary.hit_at_5_rate:.1%}")
        return 0 if report.overall_status == "pass" else 1

    if args.query:
        request = RetrievalRequest(query=args.query, top_k=args.top_k)
        result = retrieve_with_retry(request)
        if args.json:
            print(json.dumps(asdict(result), default=str, indent=2))
        else:
            print(f"Query: {result.query}")
            print(f"Results: {len(result.results)} (latency: {result.latency_ms:.1f}ms)")
            for i, chunk in enumerate(result.results, 1):
                # Encode to ascii with replace to handle special chars on Windows console
                title = chunk.page_title.encode('ascii', 'replace').decode('ascii')
                text = chunk.chunk_text[:100].encode('ascii', 'replace').decode('ascii')
                print(f"\n{i}. [{chunk.score:.3f}] {title}")
                print(f"   URL: {chunk.source_url}")
                print(f"   Text: {text}...")
        return 0

    parser.print_help()
    return 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
