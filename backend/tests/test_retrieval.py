"""Unit tests for retrieval module."""

from __future__ import annotations

import sys
import os
from unittest.mock import MagicMock, patch

import pytest

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from models import RetrievalRequest, ChunkResult, SearchResult
from errors import InvalidFilterError, InvalidQueryError, EmbeddingError
from config import FILTERABLE_FIELDS


# ============================================================================
# RetrievalRequest Tests
# ============================================================================


class TestRetrievalRequest:
    """Tests for RetrievalRequest validation."""

    def test_valid_request(self):
        """Test creating a valid request."""
        request = RetrievalRequest(query="What is ROS 2?")
        assert request.query == "What is ROS 2?"
        assert request.top_k == 5
        assert request.score_threshold is None
        assert request.filters is None

    def test_custom_top_k(self):
        """Test request with custom top_k."""
        request = RetrievalRequest(query="test", top_k=10)
        assert request.top_k == 10

    def test_with_score_threshold(self):
        """Test request with score threshold."""
        request = RetrievalRequest(query="test", score_threshold=0.5)
        assert request.score_threshold == 0.5

    def test_with_filters(self):
        """Test request with metadata filters."""
        filters = {"source_url": "https://example.com"}
        request = RetrievalRequest(query="test", filters=filters)
        assert request.filters == filters

    def test_empty_query_raises(self):
        """Test that empty query raises ValueError."""
        with pytest.raises(ValueError, match="at least 1 character"):
            RetrievalRequest(query="")

    def test_query_too_long_raises(self):
        """Test that overly long query raises ValueError."""
        long_query = "x" * 2001
        with pytest.raises(ValueError, match="at most 2000 characters"):
            RetrievalRequest(query=long_query)

    def test_invalid_top_k_low(self):
        """Test that top_k < 1 raises ValueError."""
        with pytest.raises(ValueError, match="between 1 and 100"):
            RetrievalRequest(query="test", top_k=0)

    def test_invalid_top_k_high(self):
        """Test that top_k > 100 raises ValueError."""
        with pytest.raises(ValueError, match="between 1 and 100"):
            RetrievalRequest(query="test", top_k=101)

    def test_invalid_score_threshold_low(self):
        """Test that score_threshold < 0 raises ValueError."""
        with pytest.raises(ValueError, match="between 0.0 and 1.0"):
            RetrievalRequest(query="test", score_threshold=-0.1)

    def test_invalid_score_threshold_high(self):
        """Test that score_threshold > 1 raises ValueError."""
        with pytest.raises(ValueError, match="between 0.0 and 1.0"):
            RetrievalRequest(query="test", score_threshold=1.1)


# ============================================================================
# Filter Building Tests
# ============================================================================


class TestBuildFilter:
    """Tests for build_filter function."""

    def test_none_filters_returns_none(self):
        """Test that None filters return None."""
        from retrieval import build_filter
        assert build_filter(None) is None

    def test_empty_filters_returns_none(self):
        """Test that empty dict returns None."""
        from retrieval import build_filter
        assert build_filter({}) is None

    def test_single_value_filter(self):
        """Test single value creates MatchValue condition."""
        from retrieval import build_filter
        filter_obj = build_filter({"source_url": "https://example.com"})
        assert filter_obj is not None
        assert len(filter_obj.must) == 1

    def test_list_value_filter(self):
        """Test list value creates MatchAny condition."""
        from retrieval import build_filter
        filter_obj = build_filter({"source_url": ["url1", "url2"]})
        assert filter_obj is not None
        assert len(filter_obj.must) == 1

    def test_multi_field_filter(self):
        """Test multiple fields create AND composition."""
        from retrieval import build_filter
        filter_obj = build_filter({
            "source_url": "https://example.com",
            "page_title": "Introduction",
        })
        assert filter_obj is not None
        assert len(filter_obj.must) == 2

    def test_invalid_field_raises(self):
        """Test that invalid field name raises InvalidFilterError."""
        from retrieval import build_filter
        with pytest.raises(InvalidFilterError) as exc_info:
            build_filter({"invalid_field": "value"})
        assert "invalid_field" in str(exc_info.value)

    def test_all_valid_fields(self):
        """Test all filterable fields are accepted."""
        from retrieval import build_filter
        for field in FILTERABLE_FIELDS:
            filter_obj = build_filter({field: "test_value"})
            assert filter_obj is not None


# ============================================================================
# Retrieval Tests (with mocks)
# ============================================================================


class TestRetrieve:
    """Tests for retrieve function with mocked clients."""

    @pytest.fixture
    def mock_clients(self, sample_qdrant_points):
        """Create mock Cohere and Qdrant clients."""
        # Mock Cohere client
        cohere_client = MagicMock()
        embed_response = MagicMock()
        embed_response.embeddings = [[0.1] * 1024]
        cohere_client.embed.return_value = embed_response

        # Mock Qdrant client (using query_points for qdrant-client >= 1.7)
        qdrant_client = MagicMock()
        query_response = MagicMock()
        query_response.points = sample_qdrant_points
        qdrant_client.query_points.return_value = query_response

        return cohere_client, qdrant_client

    def test_basic_retrieve(self, mock_clients, sample_qdrant_points):
        """Test basic retrieval returns SearchResult."""
        from retrieval import retrieve
        cohere_client, qdrant_client = mock_clients

        request = RetrievalRequest(query="What is ROS 2?", top_k=3)
        result = retrieve(request, cohere_client, qdrant_client)

        assert isinstance(result, SearchResult)
        assert result.query == "What is ROS 2?"
        assert len(result.results) == 3
        assert result.latency_ms > 0
        assert result.embedding_ms > 0
        assert result.search_ms > 0

    def test_results_have_metadata(self, mock_clients):
        """Test that results contain all metadata fields."""
        from retrieval import retrieve
        cohere_client, qdrant_client = mock_clients

        request = RetrievalRequest(query="test")
        result = retrieve(request, cohere_client, qdrant_client)

        for chunk in result.results:
            assert isinstance(chunk, ChunkResult)
            assert chunk.id
            assert chunk.score >= 0
            assert chunk.chunk_text
            assert chunk.source_url
            assert chunk.page_title
            # section_heading can be None
            assert isinstance(chunk.chunk_index, int)

    def test_deterministic_ordering(self, mock_clients):
        """Test that results are ordered by score DESC, id ASC."""
        from retrieval import retrieve
        cohere_client, qdrant_client = mock_clients

        request = RetrievalRequest(query="test")
        result = retrieve(request, cohere_client, qdrant_client)

        # Verify ordering
        for i in range(len(result.results) - 1):
            current = result.results[i]
            next_item = result.results[i + 1]
            # Score should be descending
            assert current.score >= next_item.score
            # If same score, id should be ascending
            if current.score == next_item.score:
                assert current.id <= next_item.id

    def test_cohere_embed_called_with_search_query(self, mock_clients):
        """Test that Cohere is called with input_type=search_query."""
        from retrieval import retrieve
        cohere_client, qdrant_client = mock_clients

        request = RetrievalRequest(query="test query")
        retrieve(request, cohere_client, qdrant_client)

        cohere_client.embed.assert_called_once()
        call_kwargs = cohere_client.embed.call_args
        assert call_kwargs[1]["input_type"] == "search_query"

    def test_qdrant_search_called_with_filter(self, mock_clients):
        """Test that Qdrant search is called with filter when provided."""
        from retrieval import retrieve
        cohere_client, qdrant_client = mock_clients

        request = RetrievalRequest(
            query="test",
            filters={"source_url": "https://example.com"},
        )
        retrieve(request, cohere_client, qdrant_client)

        qdrant_client.query_points.assert_called_once()
        call_kwargs = qdrant_client.query_points.call_args[1]
        assert call_kwargs["query_filter"] is not None


# ============================================================================
# Determinism Tests
# ============================================================================


class TestDeterminism:
    """Tests for result determinism."""

    def test_same_query_same_results(self, mock_qdrant_client, mock_cohere_client):
        """Test that identical queries return identical results."""
        from retrieval import retrieve

        # Run same query twice
        request = RetrievalRequest(query="What is ROS 2?")

        with patch("retrieval.get_cohere_client", return_value=mock_cohere_client):
            with patch("retrieval.get_qdrant_client", return_value=mock_qdrant_client):
                result1 = retrieve(request)
                result2 = retrieve(request)

        # Compare results (excluding timing)
        assert len(result1.results) == len(result2.results)
        for r1, r2 in zip(result1.results, result2.results):
            assert r1.id == r2.id
            assert r1.score == r2.score
            assert r1.chunk_text == r2.chunk_text


# ============================================================================
# Error Handling Tests
# ============================================================================


class TestErrorHandling:
    """Tests for error handling in retrieval."""

    def test_embedding_error_on_cohere_failure(self):
        """Test that Cohere failures raise EmbeddingError."""
        from retrieval import embed_query

        mock_client = MagicMock()
        mock_client.embed.side_effect = Exception("API error")

        with pytest.raises(EmbeddingError):
            embed_query("test query", mock_client)
