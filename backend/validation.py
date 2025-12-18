"""Validation and health check module for retrieval pipeline.

This module provides health checks to verify the retrieval pipeline is operational:
- Connectivity: Qdrant cluster is reachable
- Collection: Target collection exists and has vectors
- Schema: Payload fields match expected structure
- Retrieval: Basic search returns results
"""

from __future__ import annotations

import time
from datetime import datetime
from typing import Any

from qdrant_client import QdrantClient

from config import (
    QDRANT_COLLECTION,
    REQUIRED_PAYLOAD_FIELDS,
    SCHEMA_SAMPLE_SIZE,
)
from errors import (
    CollectionNotFoundError,
    QdrantConnectionError,
    SchemaValidationError,
)
from models import ValidationReport, ValidationResult


# ============================================================================
# Individual Health Checks
# ============================================================================


def check_connectivity(client: QdrantClient | None = None) -> ValidationResult:
    """Check if Qdrant cluster is reachable.

    Args:
        client: Optional QdrantClient instance

    Returns:
        ValidationResult with pass/fail status and latency
    """
    start = time.perf_counter()

    try:
        if client is None:
            from retrieval import get_qdrant_client
            client = get_qdrant_client()

        # Simple connectivity test - get cluster info
        collections = client.get_collections()
        duration_ms = (time.perf_counter() - start) * 1000

        return ValidationResult(
            name="connectivity",
            status="pass",
            message=f"Connected to Qdrant cluster ({len(collections.collections)} collections)",
            duration_ms=duration_ms,
            details={"collections_count": len(collections.collections)},
        )

    except Exception as e:
        duration_ms = (time.perf_counter() - start) * 1000
        return ValidationResult(
            name="connectivity",
            status="fail",
            message=f"Cannot connect to Qdrant: {str(e)}",
            duration_ms=duration_ms,
            details={"error": str(e)},
        )


def check_collection(client: QdrantClient | None = None) -> ValidationResult:
    """Check if target collection exists and has vectors.

    Args:
        client: Optional QdrantClient instance

    Returns:
        ValidationResult with collection info
    """
    start = time.perf_counter()

    try:
        if client is None:
            from retrieval import get_qdrant_client
            client = get_qdrant_client()

        collection_info = client.get_collection(QDRANT_COLLECTION)
        duration_ms = (time.perf_counter() - start) * 1000

        # Use points_count (vectors_count deprecated in newer qdrant-client versions)
        points_count = collection_info.points_count or 0
        indexed_vectors_count = getattr(collection_info, 'indexed_vectors_count', 0) or 0

        return ValidationResult(
            name="collection",
            status="pass",
            message=f"Collection '{QDRANT_COLLECTION}' exists with {points_count} points",
            duration_ms=duration_ms,
            details={
                "collection_name": QDRANT_COLLECTION,
                "points_count": points_count,
                "indexed_vectors_count": indexed_vectors_count,
            },
        )

    except Exception as e:
        duration_ms = (time.perf_counter() - start) * 1000
        error_str = str(e).lower()

        if "not found" in error_str or "doesn't exist" in error_str:
            return ValidationResult(
                name="collection",
                status="fail",
                message=f"Collection '{QDRANT_COLLECTION}' not found",
                duration_ms=duration_ms,
                details={"error": str(e)},
            )

        return ValidationResult(
            name="collection",
            status="fail",
            message=f"Error checking collection: {str(e)}",
            duration_ms=duration_ms,
            details={"error": str(e)},
        )


def check_schema(client: QdrantClient | None = None) -> ValidationResult:
    """Check if payload fields match expected schema.

    Samples vectors from the collection and verifies required fields are present.

    Args:
        client: Optional QdrantClient instance

    Returns:
        ValidationResult with schema validation details
    """
    start = time.perf_counter()

    try:
        if client is None:
            from retrieval import get_qdrant_client
            client = get_qdrant_client()

        # Sample vectors from collection
        points, _ = client.scroll(
            collection_name=QDRANT_COLLECTION,
            limit=SCHEMA_SAMPLE_SIZE,
            with_payload=True,
        )

        if not points:
            duration_ms = (time.perf_counter() - start) * 1000
            return ValidationResult(
                name="schema",
                status="fail",
                message="Collection is empty - cannot validate schema",
                duration_ms=duration_ms,
                details={"sampled": 0},
            )

        # Check each point for required fields
        missing_fields_by_point = []
        for point in points:
            payload = point.payload or {}
            missing = REQUIRED_PAYLOAD_FIELDS - set(payload.keys())
            if missing:
                missing_fields_by_point.append({
                    "point_id": str(point.id),
                    "missing_fields": list(missing),
                })

        duration_ms = (time.perf_counter() - start) * 1000

        if missing_fields_by_point:
            # Aggregate unique missing fields
            all_missing = set()
            for item in missing_fields_by_point:
                all_missing.update(item["missing_fields"])

            return ValidationResult(
                name="schema",
                status="fail",
                message=f"Schema validation failed: missing fields {sorted(all_missing)}",
                duration_ms=duration_ms,
                details={
                    "sampled": len(points),
                    "points_with_issues": len(missing_fields_by_point),
                    "missing_fields": sorted(all_missing),
                },
            )

        return ValidationResult(
            name="schema",
            status="pass",
            message=f"Schema valid ({len(points)} points sampled, all required fields present)",
            duration_ms=duration_ms,
            details={
                "sampled": len(points),
                "required_fields": sorted(REQUIRED_PAYLOAD_FIELDS),
            },
        )

    except Exception as e:
        duration_ms = (time.perf_counter() - start) * 1000
        return ValidationResult(
            name="schema",
            status="fail",
            message=f"Error validating schema: {str(e)}",
            duration_ms=duration_ms,
            details={"error": str(e)},
        )


def check_retrieval(client: QdrantClient | None = None) -> ValidationResult:
    """Run a smoke test to verify basic retrieval works.

    Args:
        client: Optional QdrantClient instance

    Returns:
        ValidationResult with retrieval test details
    """
    start = time.perf_counter()

    try:
        from retrieval import retrieve, get_cohere_client
        from models import RetrievalRequest

        # Use a generic test query
        test_query = "test"
        request = RetrievalRequest(query=test_query, top_k=1)

        cohere_client = get_cohere_client()
        if client is None:
            from retrieval import get_qdrant_client
            client = get_qdrant_client()

        result = retrieve(request, cohere_client, client)
        duration_ms = (time.perf_counter() - start) * 1000

        if result.results:
            return ValidationResult(
                name="retrieval",
                status="pass",
                message=f"Retrieval works ({len(result.results)} results in {result.latency_ms:.1f}ms)",
                duration_ms=duration_ms,
                details={
                    "results_count": len(result.results),
                    "embedding_ms": result.embedding_ms,
                    "search_ms": result.search_ms,
                },
            )
        else:
            return ValidationResult(
                name="retrieval",
                status="pass",
                message="Retrieval works (0 results for test query)",
                duration_ms=duration_ms,
                details={
                    "results_count": 0,
                    "note": "Empty result is valid - test query may not match content",
                },
            )

    except Exception as e:
        duration_ms = (time.perf_counter() - start) * 1000
        return ValidationResult(
            name="retrieval",
            status="fail",
            message=f"Retrieval failed: {str(e)}",
            duration_ms=duration_ms,
            details={"error": str(e)},
        )


# ============================================================================
# Validation Orchestrator
# ============================================================================


def validate(client: QdrantClient | None = None) -> ValidationReport:
    """Run all validation checks and return a comprehensive report.

    Checks are run in order, with later checks potentially skipped if
    earlier dependencies fail.

    Args:
        client: Optional QdrantClient instance (shared across checks)

    Returns:
        ValidationReport with all check results
    """
    total_start = time.perf_counter()
    timestamp = datetime.now()

    # Initialize client once for all checks
    if client is None:
        try:
            from retrieval import get_qdrant_client
            client = get_qdrant_client()
        except Exception as e:
            # If we can't get client, connectivity will fail
            pass

    checks: dict[str, ValidationResult] = {}

    # Check 1: Connectivity
    connectivity = check_connectivity(client)
    checks["connectivity"] = connectivity

    # Check 2: Collection (skip if connectivity failed)
    if connectivity.status == "pass":
        collection = check_collection(client)
        checks["collection"] = collection
    else:
        checks["collection"] = ValidationResult(
            name="collection",
            status="skip",
            message="Skipped: connectivity check failed",
            duration_ms=0,
        )

    # Check 3: Schema (skip if collection failed)
    if checks["collection"].status == "pass":
        schema = check_schema(client)
        checks["schema"] = schema
    else:
        checks["schema"] = ValidationResult(
            name="schema",
            status="skip",
            message="Skipped: collection check failed",
            duration_ms=0,
        )

    # Check 4: Retrieval (skip if schema failed)
    if checks["schema"].status == "pass":
        retrieval = check_retrieval(client)
        checks["retrieval"] = retrieval
    else:
        checks["retrieval"] = ValidationResult(
            name="retrieval",
            status="skip",
            message="Skipped: schema check failed",
            duration_ms=0,
        )

    # Calculate overall status
    total_duration_ms = (time.perf_counter() - total_start) * 1000
    overall_status = "pass" if all(c.status == "pass" for c in checks.values()) else "fail"

    return ValidationReport(
        timestamp=timestamp,
        checks=checks,
        overall_status=overall_status,
        total_duration_ms=total_duration_ms,
    )
