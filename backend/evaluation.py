"""Evaluation harness for retrieval quality metrics.

This module provides evaluation functionality:
- Load golden test queries from JSON
- Calculate MRR (Mean Reciprocal Rank)
- Calculate hit@k metrics
- Generate comprehensive evaluation reports
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

from config import GOLDEN_QUERIES_PATH, MRR_PASS_THRESHOLD
from models import (
    ChunkResult,
    EvaluationQuery,
    EvaluationQueryResult,
    EvaluationReport,
    EvaluationSummary,
    RetrievalRequest,
)


# ============================================================================
# Golden Queries Loading
# ============================================================================


def load_golden_queries(path: Path | str | None = None) -> list[EvaluationQuery]:
    """Load golden test queries from JSON file.

    Args:
        path: Path to golden queries JSON file (defaults to config path)

    Returns:
        List of EvaluationQuery objects

    Raises:
        FileNotFoundError: If golden queries file doesn't exist
        ValueError: If JSON format is invalid
    """
    if path is None:
        path = GOLDEN_QUERIES_PATH

    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Golden queries file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    queries = []
    for item in data.get("queries", []):
        query = EvaluationQuery(
            query=item["query"],
            expected_urls=item.get("expected_urls", []),
            query_type=item.get("query_type", "factual"),
        )
        queries.append(query)

    if not queries:
        raise ValueError(f"No queries found in {path}")

    return queries


# ============================================================================
# Metrics Calculation
# ============================================================================


def calculate_mrr(results: list[ChunkResult], expected_urls: list[str]) -> tuple[float, int | None]:
    """Calculate Mean Reciprocal Rank for a single query.

    MRR = 1/rank where rank is the position of the first correct result.
    Returns 0 if no correct result is found.

    Args:
        results: List of ChunkResult objects from retrieval
        expected_urls: List of expected source URLs

    Returns:
        Tuple of (mrr_score, rank) where rank is None if not found
    """
    if not expected_urls:
        # No expected URLs - can't calculate MRR
        return 0.0, None

    expected_set = set(expected_urls)

    for i, result in enumerate(results):
        # Check if result's source_url matches any expected URL
        # Use prefix matching to handle URL variations
        for expected_url in expected_set:
            if result.source_url.startswith(expected_url) or expected_url.startswith(result.source_url):
                rank = i + 1  # 1-indexed rank
                return 1.0 / rank, rank

    return 0.0, None


def hit_at_k(results: list[ChunkResult], expected_urls: list[str], k: int) -> bool:
    """Check if any correct result appears in top-k.

    Args:
        results: List of ChunkResult objects
        expected_urls: List of expected source URLs
        k: Top-k cutoff

    Returns:
        True if correct result found in top-k
    """
    if not expected_urls:
        return False

    expected_set = set(expected_urls)

    for result in results[:k]:
        for expected_url in expected_set:
            if result.source_url.startswith(expected_url) or expected_url.startswith(result.source_url):
                return True

    return False


# ============================================================================
# Evaluation Execution
# ============================================================================


def evaluate_query(
    eval_query: EvaluationQuery,
    top_k: int = 5,
) -> EvaluationQueryResult:
    """Evaluate a single query against the retrieval system.

    Args:
        eval_query: EvaluationQuery with query text and expected URLs
        top_k: Number of results to retrieve

    Returns:
        EvaluationQueryResult with metrics
    """
    from retrieval import retrieve_with_retry

    # Execute retrieval
    request = RetrievalRequest(query=eval_query.query, top_k=top_k)

    try:
        search_result = retrieve_with_retry(request)
        actual_results = search_result.results
    except Exception as e:
        # Return empty results on error
        actual_results = []

    # Calculate metrics
    mrr, rank = calculate_mrr(actual_results, eval_query.expected_urls)
    hit_1 = hit_at_k(actual_results, eval_query.expected_urls, 1)
    hit_5 = hit_at_k(actual_results, eval_query.expected_urls, 5)

    return EvaluationQueryResult(
        query=eval_query.query,
        expected_urls=eval_query.expected_urls,
        actual_results=actual_results,
        hit_at_1=hit_1,
        hit_at_5=hit_5,
        mrr=mrr,
        rank=rank,
    )


def evaluate(
    golden_queries_path: Path | str | None = None,
    top_k: int = 5,
    pass_threshold: float | None = None,
) -> EvaluationReport:
    """Run evaluation on golden test set and generate report.

    Args:
        golden_queries_path: Path to golden queries JSON
        top_k: Number of results per query
        pass_threshold: MRR threshold for pass (defaults to config)

    Returns:
        EvaluationReport with per-query and summary metrics
    """
    if pass_threshold is None:
        pass_threshold = MRR_PASS_THRESHOLD

    timestamp = datetime.now()

    # Load golden queries
    queries = load_golden_queries(golden_queries_path)

    # Evaluate each query
    results: list[EvaluationQueryResult] = []
    for query in queries:
        result = evaluate_query(query, top_k)
        results.append(result)

    # Calculate summary statistics
    total = len(results)
    queries_passed = sum(1 for r in results if r.hit_at_5)
    mrr_sum = sum(r.mrr for r in results)
    hit_1_sum = sum(1 for r in results if r.hit_at_1)
    hit_5_sum = sum(1 for r in results if r.hit_at_5)

    summary = EvaluationSummary(
        total_queries=total,
        queries_passed=queries_passed,
        mrr_average=mrr_sum / total if total > 0 else 0.0,
        hit_at_1_rate=hit_1_sum / total if total > 0 else 0.0,
        hit_at_5_rate=hit_5_sum / total if total > 0 else 0.0,
    )

    # Determine overall status
    overall_status = "pass" if summary.mrr_average >= pass_threshold else "fail"

    return EvaluationReport(
        timestamp=timestamp,
        queries_evaluated=total,
        results=results,
        summary=summary,
        overall_status=overall_status,
        pass_threshold=pass_threshold,
    )


# ============================================================================
# Standalone Metrics (for unit testing)
# ============================================================================


def calculate_mrr_from_ranks(ranks: list[int | None]) -> float:
    """Calculate average MRR from a list of ranks.

    Args:
        ranks: List of ranks (1-indexed) or None if not found

    Returns:
        Mean Reciprocal Rank (0-1)
    """
    if not ranks:
        return 0.0

    mrr_sum = sum(1.0 / r for r in ranks if r is not None and r > 0)
    return mrr_sum / len(ranks)
