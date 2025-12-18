"""End-to-end integration tests for retrieval pipeline.

These tests require live Qdrant and Cohere connections.
They are skipped if environment variables are not set.

Run with: pytest tests/integration/ -v
"""

from __future__ import annotations

import os
import sys

import pytest

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))


# Skip all tests if credentials are not available
pytestmark = pytest.mark.skipif(
    not all([
        os.environ.get("COHERE_API_KEY"),
        os.environ.get("QDRANT_URL"),
        os.environ.get("QDRANT_API_KEY"),
    ]),
    reason="Integration tests require COHERE_API_KEY, QDRANT_URL, and QDRANT_API_KEY"
)


# ============================================================================
# Live Retrieval Tests
# ============================================================================


class TestLiveRetrieval:
    """Integration tests with live Cohere and Qdrant."""

    def test_basic_retrieval(self):
        """Test basic retrieval with live services."""
        from models import RetrievalRequest
        from retrieval import retrieve_with_retry

        request = RetrievalRequest(query="What is ROS 2?", top_k=3)
        result = retrieve_with_retry(request)

        assert result is not None
        assert result.query == "What is ROS 2?"
        assert result.latency_ms > 0
        assert result.embedding_ms > 0
        assert result.search_ms > 0

    def test_retrieval_returns_metadata(self):
        """Test that results include all required metadata."""
        from models import RetrievalRequest
        from retrieval import retrieve_with_retry

        request = RetrievalRequest(query="ROS 2 nodes", top_k=1)
        result = retrieve_with_retry(request)

        if result.results:
            chunk = result.results[0]
            assert chunk.id
            assert chunk.score >= 0
            assert chunk.chunk_text
            assert chunk.source_url
            assert chunk.page_title
            # section_heading can be None
            assert isinstance(chunk.chunk_index, int)

    def test_retrieval_determinism(self):
        """Test that same query returns same results."""
        from models import RetrievalRequest
        from retrieval import retrieve_with_retry

        request = RetrievalRequest(query="robot operating system", top_k=5)

        result1 = retrieve_with_retry(request)
        result2 = retrieve_with_retry(request)

        assert len(result1.results) == len(result2.results)
        for r1, r2 in zip(result1.results, result2.results):
            assert r1.id == r2.id
            assert r1.score == r2.score

    def test_retrieval_with_filter(self):
        """Test retrieval with metadata filter."""
        from models import RetrievalRequest
        from retrieval import retrieve_with_retry

        # First, get a URL from an unfiltered search
        request_unfiltered = RetrievalRequest(query="ROS", top_k=1)
        result = retrieve_with_retry(request_unfiltered)

        if result.results:
            target_url = result.results[0].source_url

            # Now filter by that URL
            request_filtered = RetrievalRequest(
                query="ROS",
                top_k=5,
                filters={"source_url": target_url},
            )
            filtered_result = retrieve_with_retry(request_filtered)

            # All results should match the filter
            for chunk in filtered_result.results:
                assert chunk.source_url == target_url


# ============================================================================
# Live Validation Tests
# ============================================================================


class TestLiveValidation:
    """Integration tests for validation with live services."""

    def test_validate_passes(self):
        """Test that validation passes with live services."""
        from validation import validate

        report = validate()

        assert report is not None
        assert report.overall_status in ["pass", "fail"]
        assert "connectivity" in report.checks
        assert "collection" in report.checks
        assert "schema" in report.checks
        assert "retrieval" in report.checks

    def test_connectivity_check(self):
        """Test connectivity check with live Qdrant."""
        from validation import check_connectivity

        result = check_connectivity()

        assert result.status == "pass"
        assert result.duration_ms > 0

    def test_collection_check(self):
        """Test collection check with live Qdrant."""
        from validation import check_collection

        result = check_collection()

        assert result.status == "pass"
        assert "points" in result.message.lower()

    def test_schema_check(self):
        """Test schema validation with live data."""
        from validation import check_schema

        result = check_schema()

        assert result.status == "pass"
        assert result.details.get("sampled", 0) > 0


# ============================================================================
# Live Evaluation Tests
# ============================================================================


class TestLiveEvaluation:
    """Integration tests for evaluation with live services."""

    def test_evaluate_runs(self):
        """Test that evaluation completes with live services."""
        from evaluation import evaluate

        report = evaluate()

        assert report is not None
        assert report.queries_evaluated > 0
        assert report.summary.total_queries > 0
        assert report.overall_status in ["pass", "fail"]

    def test_mrr_is_calculated(self):
        """Test that MRR is calculated correctly."""
        from evaluation import evaluate

        report = evaluate()

        assert 0.0 <= report.summary.mrr_average <= 1.0
        assert 0.0 <= report.summary.hit_at_1_rate <= 1.0
        assert 0.0 <= report.summary.hit_at_5_rate <= 1.0


# ============================================================================
# Performance Tests
# ============================================================================


class TestPerformance:
    """Performance-related integration tests."""

    def test_query_latency_under_threshold(self):
        """Test that query latency is under 5 seconds."""
        from models import RetrievalRequest
        from retrieval import retrieve_with_retry

        request = RetrievalRequest(query="What is ROS?", top_k=5)
        result = retrieve_with_retry(request)

        # Total latency should be under 5000ms
        assert result.latency_ms < 5000

    def test_embedding_latency_under_threshold(self):
        """Test that embedding latency is under 2 seconds."""
        from models import RetrievalRequest
        from retrieval import retrieve_with_retry

        request = RetrievalRequest(query="Robot Operating System", top_k=1)
        result = retrieve_with_retry(request)

        # Embedding should be under 2000ms
        assert result.embedding_ms < 2000

    def test_search_latency_under_threshold(self):
        """Test that Qdrant search is under 1 second."""
        from models import RetrievalRequest
        from retrieval import retrieve_with_retry

        request = RetrievalRequest(query="test query", top_k=10)
        result = retrieve_with_retry(request)

        # Search should be under 1000ms
        assert result.search_ms < 1000
