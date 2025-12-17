"""Unit tests for validation module."""

from __future__ import annotations

import sys
import os
from unittest.mock import MagicMock, patch

import pytest

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from models import ValidationResult, ValidationReport


# ============================================================================
# Check Connectivity Tests
# ============================================================================


class TestCheckConnectivity:
    """Tests for check_connectivity function."""

    def test_success_with_mock_client(self):
        """Test connectivity check passes with working client."""
        from validation import check_connectivity

        mock_client = MagicMock()
        mock_collections = MagicMock()
        mock_collections.collections = [MagicMock(), MagicMock()]
        mock_client.get_collections.return_value = mock_collections

        result = check_connectivity(mock_client)

        assert result.status == "pass"
        assert result.name == "connectivity"
        assert result.duration_ms > 0
        assert "2 collections" in result.message

    def test_failure_on_connection_error(self):
        """Test connectivity check fails on connection error."""
        from validation import check_connectivity

        mock_client = MagicMock()
        mock_client.get_collections.side_effect = Exception("Connection refused")

        result = check_connectivity(mock_client)

        assert result.status == "fail"
        assert "Connection refused" in result.message


# ============================================================================
# Check Collection Tests
# ============================================================================


class TestCheckCollection:
    """Tests for check_collection function."""

    def test_success_with_existing_collection(self):
        """Test collection check passes when collection exists."""
        from validation import check_collection

        mock_client = MagicMock()
        mock_info = MagicMock()
        mock_info.vectors_count = 150
        mock_info.points_count = 150
        mock_client.get_collection.return_value = mock_info

        result = check_collection(mock_client)

        assert result.status == "pass"
        assert "150 points" in result.message
        assert result.details["points_count"] == 150

    def test_failure_when_collection_not_found(self):
        """Test collection check fails when collection doesn't exist."""
        from validation import check_collection

        mock_client = MagicMock()
        mock_client.get_collection.side_effect = Exception("Collection not found")

        result = check_collection(mock_client)

        assert result.status == "fail"
        assert "not found" in result.message.lower()


# ============================================================================
# Check Schema Tests
# ============================================================================


class TestCheckSchema:
    """Tests for check_schema function."""

    def test_success_with_valid_schema(self, sample_qdrant_points):
        """Test schema check passes with valid payload fields."""
        from validation import check_schema

        mock_client = MagicMock()
        mock_client.scroll.return_value = (sample_qdrant_points, None)

        result = check_schema(mock_client)

        assert result.status == "pass"
        assert "valid" in result.message.lower()
        assert result.details["sampled"] == len(sample_qdrant_points)

    def test_failure_with_missing_fields(self):
        """Test schema check fails when required fields are missing."""
        from validation import check_schema

        # Create points with missing fields
        mock_point = MagicMock()
        mock_point.id = "test-id"
        mock_point.payload = {"chunk_text": "some text"}  # Missing other fields

        mock_client = MagicMock()
        mock_client.scroll.return_value = ([mock_point], None)

        result = check_schema(mock_client)

        assert result.status == "fail"
        assert "missing" in result.message.lower()

    def test_failure_when_collection_empty(self):
        """Test schema check fails when collection is empty."""
        from validation import check_schema

        mock_client = MagicMock()
        mock_client.scroll.return_value = ([], None)

        result = check_schema(mock_client)

        assert result.status == "fail"
        assert "empty" in result.message.lower()


# ============================================================================
# Check Retrieval Tests
# ============================================================================


class TestCheckRetrieval:
    """Tests for check_retrieval function."""

    def test_success_with_results(self, sample_qdrant_points):
        """Test retrieval check passes when search returns results."""
        from validation import check_retrieval

        mock_qdrant = MagicMock()
        mock_response = MagicMock()
        mock_response.points = sample_qdrant_points
        mock_qdrant.query_points.return_value = mock_response

        mock_cohere = MagicMock()
        mock_embed_response = MagicMock()
        mock_embed_response.embeddings = [[0.1] * 1024]
        mock_cohere.embed.return_value = mock_embed_response

        with patch("retrieval.get_cohere_client", return_value=mock_cohere):
            result = check_retrieval(mock_qdrant)

        assert result.status == "pass"
        assert "works" in result.message.lower()

    def test_success_with_empty_results(self):
        """Test retrieval check passes even with zero results."""
        from validation import check_retrieval

        mock_qdrant = MagicMock()
        mock_response = MagicMock()
        mock_response.points = []
        mock_qdrant.query_points.return_value = mock_response

        mock_cohere = MagicMock()
        mock_embed_response = MagicMock()
        mock_embed_response.embeddings = [[0.1] * 1024]
        mock_cohere.embed.return_value = mock_embed_response

        with patch("retrieval.get_cohere_client", return_value=mock_cohere):
            result = check_retrieval(mock_qdrant)

        assert result.status == "pass"
        assert "0 results" in result.message

    def test_failure_on_error(self):
        """Test retrieval check fails on error."""
        from validation import check_retrieval

        mock_qdrant = MagicMock()
        mock_qdrant.query_points.side_effect = Exception("Search failed")

        mock_cohere = MagicMock()
        mock_embed_response = MagicMock()
        mock_embed_response.embeddings = [[0.1] * 1024]
        mock_cohere.embed.return_value = mock_embed_response

        with patch("retrieval.get_cohere_client", return_value=mock_cohere):
            result = check_retrieval(mock_qdrant)

        assert result.status == "fail"


# ============================================================================
# Validate Orchestrator Tests
# ============================================================================


class TestValidate:
    """Tests for validate orchestrator function."""

    def test_all_checks_pass(self, sample_qdrant_points):
        """Test validation report when all checks pass."""
        from validation import validate

        mock_client = MagicMock()

        # Setup for connectivity
        mock_collections = MagicMock()
        mock_collections.collections = [MagicMock()]
        mock_client.get_collections.return_value = mock_collections

        # Setup for collection
        mock_info = MagicMock()
        mock_info.points_count = 100
        mock_info.indexed_vectors_count = 100
        mock_client.get_collection.return_value = mock_info

        # Setup for schema
        mock_client.scroll.return_value = (sample_qdrant_points, None)

        # Setup for retrieval (using query_points)
        mock_response = MagicMock()
        mock_response.points = sample_qdrant_points
        mock_client.query_points.return_value = mock_response

        mock_cohere = MagicMock()
        mock_embed_response = MagicMock()
        mock_embed_response.embeddings = [[0.1] * 1024]
        mock_cohere.embed.return_value = mock_embed_response

        with patch("retrieval.get_cohere_client", return_value=mock_cohere):
            report = validate(mock_client)

        assert isinstance(report, ValidationReport)
        assert report.overall_status == "pass"
        assert len(report.checks) == 4
        assert all(c.status == "pass" for c in report.checks.values())

    def test_skip_checks_on_connectivity_failure(self):
        """Test that later checks are skipped when connectivity fails."""
        from validation import validate

        mock_client = MagicMock()
        mock_client.get_collections.side_effect = Exception("Connection failed")

        report = validate(mock_client)

        assert report.overall_status == "fail"
        assert report.checks["connectivity"].status == "fail"
        assert report.checks["collection"].status == "skip"
        assert report.checks["schema"].status == "skip"
        assert report.checks["retrieval"].status == "skip"

    def test_skip_schema_on_collection_failure(self):
        """Test that schema check is skipped when collection check fails."""
        from validation import validate

        mock_client = MagicMock()

        # Connectivity passes
        mock_collections = MagicMock()
        mock_collections.collections = [MagicMock()]
        mock_client.get_collections.return_value = mock_collections

        # Collection fails
        mock_client.get_collection.side_effect = Exception("Collection not found")

        report = validate(mock_client)

        assert report.overall_status == "fail"
        assert report.checks["connectivity"].status == "pass"
        assert report.checks["collection"].status == "fail"
        assert report.checks["schema"].status == "skip"
        assert report.checks["retrieval"].status == "skip"

    def test_report_has_timing_info(self, sample_qdrant_points):
        """Test that validation report includes timing information."""
        from validation import validate

        mock_client = MagicMock()
        mock_collections = MagicMock()
        mock_collections.collections = []
        mock_client.get_collections.return_value = mock_collections
        mock_info = MagicMock()
        mock_info.points_count = 10
        mock_info.indexed_vectors_count = 10
        mock_client.get_collection.return_value = mock_info
        mock_client.scroll.return_value = (sample_qdrant_points, None)

        # Setup for retrieval (using query_points)
        mock_response = MagicMock()
        mock_response.points = sample_qdrant_points
        mock_client.query_points.return_value = mock_response

        mock_cohere = MagicMock()
        mock_embed_response = MagicMock()
        mock_embed_response.embeddings = [[0.1] * 1024]
        mock_cohere.embed.return_value = mock_embed_response

        with patch("retrieval.get_cohere_client", return_value=mock_cohere):
            report = validate(mock_client)

        assert report.total_duration_ms > 0
        for check in report.checks.values():
            if check.status != "skip":
                assert check.duration_ms >= 0
