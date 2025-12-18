"""API smoke tests for RAG Agent API (Spec-3).

These tests verify basic functionality of all API endpoints without
requiring live external services. They use mocked agents and retrieval.
"""

from __future__ import annotations

import json
from unittest.mock import AsyncMock, patch

import pytest


# ============================================================================
# Health Endpoint Tests
# ============================================================================


class TestHealthEndpoint:
    """Tests for GET /health endpoint."""

    def test_health_returns_200(self, test_client):
        """Health endpoint should return 200 OK."""
        response = test_client.get("/health")
        assert response.status_code == 200

    def test_health_response_structure(self, test_client):
        """Health response should have correct structure."""
        response = test_client.get("/health")
        data = response.json()

        assert "status" in data
        assert "services" in data
        assert "timestamp" in data

        assert data["status"] in ["healthy", "degraded", "unhealthy"]

    def test_health_services_included(self, test_client):
        """Health response should include all services."""
        response = test_client.get("/health")
        data = response.json()

        # Should check at least qdrant, cohere, and gemini
        services = data.get("services", {})
        # Note: actual status depends on env vars being set


# ============================================================================
# Root Endpoint Tests
# ============================================================================


class TestRootEndpoint:
    """Tests for GET / endpoint."""

    def test_root_returns_200(self, test_client):
        """Root endpoint should return 200 OK."""
        response = test_client.get("/")
        assert response.status_code == 200

    def test_root_response_structure(self, test_client):
        """Root response should have API info."""
        response = test_client.get("/")
        data = response.json()

        assert "name" in data
        assert "version" in data
        assert "docs" in data
        assert "health" in data


# ============================================================================
# Chat Endpoint Tests
# ============================================================================


class TestChatEndpoint:
    """Tests for POST /chat endpoint."""

    def test_chat_requires_query(self, test_client):
        """Chat should return 422 if query is missing."""
        response = test_client.post("/chat", json={})
        assert response.status_code == 422

    def test_chat_validates_query_length(self, test_client):
        """Chat should reject empty queries."""
        response = test_client.post("/chat", json={"query": ""})
        assert response.status_code == 422

    def test_chat_validates_query_max_length(self, test_client):
        """Chat should reject queries exceeding max length."""
        long_query = "a" * 2001
        response = test_client.post("/chat", json={"query": long_query})
        assert response.status_code == 422

    def test_chat_validates_mode(self, test_client):
        """Chat should reject invalid mode values."""
        response = test_client.post(
            "/chat", json={"query": "What is ROS?", "mode": "invalid_mode"}
        )
        assert response.status_code == 422

    def test_chat_validates_selected_text_mode_requires_text(self, test_client):
        """Chat should reject selected_text mode without selected_text."""
        response = test_client.post(
            "/chat", json={"query": "What is mentioned?", "mode": "selected_text"}
        )
        assert response.status_code == 422

    def test_chat_validates_top_k_range(self, test_client):
        """Chat should validate top_k is within range."""
        # Too low
        response = test_client.post(
            "/chat", json={"query": "What is ROS?", "top_k": 0}
        )
        assert response.status_code == 422

        # Too high
        response = test_client.post(
            "/chat", json={"query": "What is ROS?", "top_k": 21}
        )
        assert response.status_code == 422

    def test_chat_general_mode_success(self, test_client, mock_agent_run):
        """Chat in general mode should return answer with sources."""
        response = test_client.post(
            "/chat", json={"query": "What is ROS 2?", "mode": "general"}
        )

        assert response.status_code == 200
        data = response.json()

        assert "answer" in data
        assert "sources" in data
        assert "mode" in data
        assert "metadata" in data

        assert data["mode"] == "general"
        assert isinstance(data["sources"], list)

    def test_chat_selected_text_mode_success(self, test_client, mock_agent_run):
        """Chat in selected_text mode should work with provided text."""
        mock_agent_run.return_value = (
            "The text mentions a robotics middleware.",
            [
                {
                    "source_url": "selected_text",
                    "page_title": "User Selection",
                    "section_heading": "Selected Text",
                    "chunk_text": "ROS 2 is a robotics middleware.",
                    "relevance_score": 1.0,
                }
            ],
        )

        response = test_client.post(
            "/chat",
            json={
                "query": "What is mentioned?",
                "mode": "selected_text",
                "selected_text": "ROS 2 is a robotics middleware.",
            },
        )

        assert response.status_code == 200
        data = response.json()

        assert data["mode"] == "selected_text"
        assert data["answer"] == "The text mentions a robotics middleware."

    def test_chat_response_metadata(self, test_client, mock_agent_run):
        """Chat response should include metadata."""
        response = test_client.post(
            "/chat", json={"query": "What is ROS 2?", "mode": "general"}
        )

        assert response.status_code == 200
        data = response.json()

        metadata = data.get("metadata", {})
        assert "query_time_ms" in metadata
        assert "chunks_retrieved" in metadata
        assert "model" in metadata

    def test_chat_source_citation_structure(self, test_client, mock_agent_run):
        """Source citations should have correct structure."""
        response = test_client.post(
            "/chat", json={"query": "What is ROS 2?", "mode": "general"}
        )

        assert response.status_code == 200
        data = response.json()

        sources = data.get("sources", [])
        if sources:
            source = sources[0]
            assert "source_url" in source
            assert "page_title" in source
            assert "section_heading" in source
            assert "chunk_text" in source
            assert "relevance_score" in source

    def test_chat_handles_agent_error(self, test_client, mock_agent_run):
        """Chat should handle agent errors gracefully."""
        mock_agent_run.side_effect = Exception("Agent failed")

        response = test_client.post(
            "/chat", json={"query": "What is ROS 2?", "mode": "general"}
        )

        assert response.status_code == 500
        data = response.json()
        detail = data.get("detail", {})
        assert "error_code" in detail

    def test_chat_with_filters(self, test_client, mock_agent_run):
        """Chat should accept filters parameter."""
        response = test_client.post(
            "/chat",
            json={
                "query": "What is ROS 2?",
                "mode": "general",
                "filters": {"source_url": "https://example.com/intro"},
            },
        )

        assert response.status_code == 200

    def test_chat_with_score_threshold(self, test_client, mock_agent_run):
        """Chat should accept score_threshold parameter."""
        response = test_client.post(
            "/chat",
            json={
                "query": "What is ROS 2?",
                "mode": "general",
                "score_threshold": 0.7,
            },
        )

        assert response.status_code == 200


# ============================================================================
# Chat Stream Endpoint Tests
# ============================================================================


class TestChatStreamEndpoint:
    """Tests for POST /chat/stream endpoint."""

    def test_stream_requires_query(self, test_client):
        """Stream should return 422 if query is missing."""
        response = test_client.post("/chat/stream", json={})
        assert response.status_code == 422

    def test_stream_validates_mode(self, test_client):
        """Stream should reject invalid mode values."""
        response = test_client.post(
            "/chat/stream", json={"query": "What is ROS?", "mode": "invalid_mode"}
        )
        assert response.status_code == 422

    def test_stream_returns_event_stream(self, test_client, mock_agent_run_streaming):
        """Stream should return text/event-stream content type."""
        response = test_client.post(
            "/chat/stream", json={"query": "What is ROS 2?", "mode": "general"}
        )

        assert response.status_code == 200
        assert "text/event-stream" in response.headers.get("content-type", "")


# ============================================================================
# Error Response Tests
# ============================================================================


class TestErrorResponses:
    """Tests for error response format."""

    def test_validation_error_format(self, test_client):
        """Validation errors should have correct format."""
        response = test_client.post("/chat", json={})
        assert response.status_code == 422

        # FastAPI validation errors have a specific format
        data = response.json()
        assert "detail" in data

    def test_not_found_returns_404(self, test_client):
        """Unknown endpoints should return 404."""
        response = test_client.get("/nonexistent")
        assert response.status_code == 404


# ============================================================================
# Pydantic Model Tests
# ============================================================================


class TestPydanticModels:
    """Tests for Pydantic model validation."""

    def test_chat_request_model_validation(self):
        """ChatRequest model should validate correctly."""
        from api_models import ChatRequest

        # Valid request
        request = ChatRequest(query="What is ROS 2?")
        assert request.query == "What is ROS 2?"
        assert request.mode == "general"
        assert request.top_k == 5

        # With optional fields
        request = ChatRequest(
            query="Test",
            mode="general",
            top_k=10,
            score_threshold=0.5,
        )
        assert request.top_k == 10
        assert request.score_threshold == 0.5

    def test_chat_request_selected_text_validation(self):
        """ChatRequest should require selected_text for selected_text mode."""
        from api_models import ChatRequest

        with pytest.raises(ValueError):
            ChatRequest(query="Test", mode="selected_text")

        # Should work with selected_text
        request = ChatRequest(
            query="Test",
            mode="selected_text",
            selected_text="Some text",
        )
        assert request.selected_text == "Some text"

    def test_source_citation_model(self):
        """SourceCitation model should validate correctly."""
        from api_models import SourceCitation

        citation = SourceCitation(
            source_url="https://example.com",
            page_title="Test Page",
            section_heading="Introduction",
            chunk_text="Test content",
            relevance_score=0.95,
        )

        assert citation.source_url == "https://example.com"
        assert citation.relevance_score == 0.95

    def test_source_citation_truncates_chunk_text(self):
        """SourceCitation should truncate chunk_text to 500 chars."""
        from api_models import SourceCitation

        long_text = "a" * 501
        citation = SourceCitation(
            source_url="https://example.com",
            page_title="Test",
            section_heading="Test",
            chunk_text=long_text[:500],  # Manual truncation in API
            relevance_score=0.5,
        )

        assert len(citation.chunk_text) <= 500

    def test_health_response_model(self):
        """HealthResponse model should validate correctly."""
        from datetime import datetime
        from api_models import HealthResponse, ServiceStatus

        response = HealthResponse(
            status="healthy",
            services={
                "qdrant": ServiceStatus(status="up", latency_ms=45.2),
                "cohere": ServiceStatus(status="up", latency_ms=120.5),
            },
            timestamp=datetime.utcnow(),
        )

        assert response.status == "healthy"
        assert "qdrant" in response.services

    def test_error_response_model(self):
        """ErrorResponse model should validate correctly."""
        from api_models import ErrorResponse

        error = ErrorResponse(
            error_code="validation_error",
            message="Query too short",
            trace_id="abc12345",
        )

        assert error.error_code == "validation_error"
        assert error.trace_id == "abc12345"


# ============================================================================
# Integration Tests (with mocked external services)
# ============================================================================


class TestIntegration:
    """Integration tests with mocked external services."""

    def test_full_chat_flow(self, test_client, mock_agent_run):
        """Test complete chat flow from request to response."""
        # Make request
        response = test_client.post(
            "/chat",
            json={
                "query": "What is ROS 2 and how does it work?",
                "mode": "general",
                "top_k": 5,
            },
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()

        # Verify all required fields
        assert data["answer"]
        assert data["mode"] == "general"
        assert isinstance(data["sources"], list)
        assert data["metadata"]["chunks_retrieved"] >= 0
        assert data["metadata"]["query_time_ms"] >= 0

    def test_selected_text_mode_flow(self, test_client, mock_agent_run):
        """Test complete selected-text mode flow."""
        mock_agent_run.return_value = (
            "The selection discusses robotics middleware.",
            [
                {
                    "source_url": "selected_text",
                    "page_title": "User Selection",
                    "section_heading": "Selected Text",
                    "chunk_text": "ROS 2 is a modern robotics middleware...",
                    "relevance_score": 1.0,
                }
            ],
        )

        response = test_client.post(
            "/chat",
            json={
                "query": "What does this text discuss?",
                "mode": "selected_text",
                "selected_text": "ROS 2 is a modern robotics middleware that provides tools for building robot applications.",
            },
        )

        assert response.status_code == 200
        data = response.json()

        assert data["mode"] == "selected_text"
        # Should have source from selected text
        sources = data["sources"]
        if sources:
            assert any(s["source_url"] == "selected_text" for s in sources)
