"""Data models for retrieval pipeline.

This module defines all dataclasses used across the retrieval, validation,
and evaluation modules. All models support JSON serialization via dataclasses.asdict().
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Literal


# ============================================================================
# Core Retrieval Models
# ============================================================================


@dataclass
class RetrievalRequest:
    """Request for semantic similarity search.

    Attributes:
        query: Natural language query (1-2000 chars)
        top_k: Number of results to return (1-100, default 10)
        score_threshold: Minimum similarity score (0.0-1.0, optional)
        filters: Metadata filters (source_url, page_title, section_heading)
    """

    query: str
    top_k: int = 10
    score_threshold: float | None = None
    filters: dict[str, str | list[str]] | None = None

    def __post_init__(self):
        """Validate request parameters."""
        from config import QUERY_MIN_LENGTH, QUERY_MAX_LENGTH, K_MIN, K_MAX

        if not self.query or len(self.query) < QUERY_MIN_LENGTH:
            raise ValueError(f"Query must be at least {QUERY_MIN_LENGTH} character(s)")
        if len(self.query) > QUERY_MAX_LENGTH:
            raise ValueError(f"Query must be at most {QUERY_MAX_LENGTH} characters")
        if self.top_k < K_MIN or self.top_k > K_MAX:
            raise ValueError(f"top_k must be between {K_MIN} and {K_MAX}")
        if self.score_threshold is not None:
            if not 0.0 <= self.score_threshold <= 1.0:
                raise ValueError("score_threshold must be between 0.0 and 1.0")


@dataclass
class ChunkResult:
    """Single chunk result from retrieval.

    Attributes:
        id: Qdrant point ID (32-char hex)
        score: Similarity score (0.0-1.0)
        chunk_text: The text content
        source_url: Source page URL
        page_title: Page title
        section_heading: Section heading (may be None)
        chunk_index: Position in document (0-based)
    """

    id: str
    score: float
    chunk_text: str
    source_url: str
    page_title: str
    section_heading: str | None
    chunk_index: int


@dataclass
class SearchResult:
    """Complete search result with metadata.

    Attributes:
        query: Original query string
        results: Ranked list of ChunkResult objects
        total_found: Total matches before top_k limit
        latency_ms: Total query execution time
        embedding_ms: Cohere API call duration
        search_ms: Qdrant search duration
    """

    query: str
    results: list[ChunkResult]
    total_found: int
    latency_ms: float
    embedding_ms: float
    search_ms: float


# ============================================================================
# Validation Models
# ============================================================================


@dataclass
class ValidationResult:
    """Result of a single validation check.

    Attributes:
        name: Check name (e.g., "connectivity")
        status: Pass, fail, or skip status
        message: Human-readable message
        duration_ms: Check duration in milliseconds
        details: Additional context (optional)
    """

    name: str
    status: Literal["pass", "fail", "skip"]
    message: str
    duration_ms: float
    details: dict[str, Any] | None = None


@dataclass
class ValidationReport:
    """Complete validation suite report.

    Attributes:
        timestamp: When validation was run
        checks: Dictionary of check name to ValidationResult
        overall_status: Overall pass/fail status
        total_duration_ms: Total validation duration
    """

    timestamp: datetime
    checks: dict[str, ValidationResult]
    overall_status: Literal["pass", "fail"]
    total_duration_ms: float


# ============================================================================
# Evaluation Models
# ============================================================================


@dataclass
class EvaluationQuery:
    """Single query for evaluation.

    Attributes:
        query: The query text
        expected_urls: Expected source URLs for correct results
        query_type: Type of query (factual, conceptual, procedural)
    """

    query: str
    expected_urls: list[str]
    query_type: Literal["factual", "conceptual", "procedural"]


@dataclass
class EvaluationQueryResult:
    """Result for a single evaluation query.

    Attributes:
        query: The query text
        expected_urls: Expected source URLs
        actual_results: Retrieved ChunkResult objects
        hit_at_1: True if correct result at position 1
        hit_at_5: True if correct result in top 5
        mrr: Reciprocal rank for this query
        rank: Position of first correct result (None if not found)
    """

    query: str
    expected_urls: list[str] | None
    actual_results: list[ChunkResult]
    hit_at_1: bool
    hit_at_5: bool
    mrr: float
    rank: int | None


@dataclass
class EvaluationSummary:
    """Summary statistics for evaluation.

    Attributes:
        total_queries: Number of queries evaluated
        queries_passed: Number with hit_at_5 == True
        mrr_average: Mean Reciprocal Rank across all queries
        hit_at_1_rate: Percentage with correct top result
        hit_at_5_rate: Percentage with correct in top 5
    """

    total_queries: int
    queries_passed: int
    mrr_average: float
    hit_at_1_rate: float
    hit_at_5_rate: float


@dataclass
class EvaluationReport:
    """Complete evaluation report.

    Attributes:
        timestamp: When evaluation was run
        queries_evaluated: Number of queries processed
        results: Per-query evaluation results
        summary: Aggregated statistics
        overall_status: Pass if MRR >= threshold
        pass_threshold: MRR threshold for passing (default 0.5)
    """

    timestamp: datetime
    queries_evaluated: int
    results: list[EvaluationQueryResult]
    summary: EvaluationSummary
    overall_status: Literal["pass", "fail"]
    pass_threshold: float = 0.5
